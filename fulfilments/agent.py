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

logging.basicConfig(level=logging.DEBUG)


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
    def __init__(self, apikey=None):

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
        self.url = 'https://api.dialogflow.com/v1/query'

    def query(self, q, robot=str(uuid4())):
        data = {
            'contexts': self.contexts,
            'query': q,
            'lang': 'en',
            'sessionId': str(uuid4())
        }
        logging.info(pformat(data))
        r = post(self.url, data=dumps(data), headers=self.header)
        fparams = r.json()
        logging.info(pformat(fparams))
        # fulfilment = SimulationDispatcher()
        # fulfilment.robot = robot
        # ff_response = fulfilment.dispatch(fparams)
        self.contexts = fparams['result']['metadata']['contexts']
        # if ff_response['speech'] is not "":
        #     self.contexts = ff_response['contextOut']
        #     fparams['result'] = ff_response
        # print "***" + pformat(fparams)
        return fparams
        #logging.info(pformat(r.json()))

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
    agent = Agent()
    agent.query("what is bielefeld?")
