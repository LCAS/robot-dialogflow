#!/usr/bin/env python
import web

from uuid import uuid4

import logging
from os import getenv
from json import loads, dumps
from pprint import pformat
from requests import post
from collections import defaultdict
from os import _exit
import signal
from time import time
from urllib import quote

from simulation import SimulationDispatcher

logging.basicConfig(level=logging.INFO)


class SessionManager:
    '''
    A management class containing all active sessions
    '''
    def __init__(self):
        self.__active_sessions = defaultdict(dict)

    def get(self, session_id):
        return self.__active_sessions[session_id]

    def all(self):
        return dict(self.__active_sessions)

    def set(self, session_id, args={}):
        self.__active_sessions[session_id] = args


session_manager = SessionManager()


class Agent:
    '''
    Class to send dialogflow event out via the standard API
    '''
    def __init__(self, apikey=None,robot=None,session=None):

        self.contexts = []
        if apikey is not None:
            self.apikey = apikey
        else:
            self.apikey = getenv('DF_APIKEY', 'invalidapikey')
            logging.info('APIKEY: %s' % self.apikey)
        self.header = {
            'Authorization': 'Bearer %s' % self.apikey,
            'Content-Type': 'application/json; charset=utf-8'
        }
        self.url = 'https://api.dialogflow.com/v1/query?v=20170712'

        if robot:
            self.robot = robot
        else:
            self.robot = str(uuid4())
        if session:
            self.session = session
        else:
            self.session = str(uuid4())
        self.self_fulfilment = False

    def query(self, q, session=None, robot=None):
        if session is None:
            session = self.session
        if robot is None:
            robot = self.robot
        data = {
            'contexts': self.contexts,
            'query': q,
            'lang': 'en',
            'sessionId': session
        }
        logging.info('request data: %s' % pformat(data))
        r = post(self.url, data=dumps(data), headers=self.header)
        response = r.json()
        logging.info(pformat(response))

        result = response['result']
        if response['status']['code'] is not 200:
            logging.warning('FAILED: %s' % pformat(response['status']))
        if self.self_fulfilment:
            action = result.get('action')
            actionIncomplete = result.get('actionIncomplete', False)
            #self.contexts = result['metadata']['contexts']

            if action is not None and len(action) > 0:
                logging.info('found action: "%s"' % action)
                fulfilment = SimulationDispatcher()
                fulfilment.robot = robot
                if not actionIncomplete and fulfilment.can_dispatch(action):
                    result = fulfilment.dispatch(response)
                    logging.info('action %s completed' % action)

            contextsOut = result.get('contextOut', None)
            if contextsOut:
                self.contexts = result['contextOut']
            logging.info('final result: %s' % pformat(result))
        return result

    def send_event(self, event_name, parameters={}, sessionId=None):
        if sessionId is not None:
            return self._send_event(event_name, parameters, sessionId)
        else:
            return [
                self._send_event(event_name, parameters, s)
                for s in session_manager.all()
            ]

    def _send_event(self, event_name, parameters, sessionId):
        logging.info('send event %s to session %s' % (event_name, sessionId))
        data = {
            'event': {
                'name': event_name,
                'data': parameters
            },
            'lang': 'en',
            'sessionId': sessionId

        }
        r = post(self.url, data=dumps(data), headers=self.header)
        logging.info(pformat(r.json()))


if __name__ == "__main__":
    agent = Agent(robot="hurga")
    #agent.query("what is bielefeld?")
    #agent.query("go to the kitchen")
    agent.query("go away")
    agent.query("kitchen")
    #agent.query("where are you?")
