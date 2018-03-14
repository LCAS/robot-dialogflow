#!/usr/bin/env python
import web

import logging
from os import getenv
from json import loads, dumps
from pprint import pformat
from requests import post
from collections import defaultdict
from os import _exit
import signal

logging.basicConfig(level=logging.DEBUG)

try:
    import rospy
    from actionlib import SimpleActionClient
    from topological_navigation.msg import GotoNode
    from mary_tts.msg import marytts
    ros_available = True
    logging.info('running in ROS mode')
except:
    ros_available = False
    logging.info('running without ROS')


urls = (
    '/simrob/webhook/(.+)', 'index',
    '/simrob/(.+)', 'test'
)


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


class EventDispatcher:
    '''
    Class to send dialogflow event out via the standard API
    '''
    def __init__(self, apikey=None):
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


class ROSInterface:

    def __init__(self):
        rospy.init_node('dialogflow_fulfilment')
        self.goto_action_server = SimpleActionClient(
            '/topological_navigation',
            GotoNode)
        self.speak_action_server = SimpleActionClient(
            '/speak',
            marytts)


class SimulationInterface:

    def __init__(self):
        self.locations = defaultdict(str)
        self.utterances = defaultdict(list)

    def get_location(self, robot):
        return self.locations[robot]

    def set_location(self, robot, location):
        self.locations[robot] = location

    def add_utterance(self, robot, utterance):
        self.utterances[robot].append(utterance)

    def get_utterances(self, robot):
        return self.utterances[robot]


simulation = SimulationInterface()


if ros_available:
    try:
        ros = ROSInterface()
    except:
        logging.warn("couldn't initialise ROS, working without")
        ros = None
else:
    ros = None


class FulfilmentDispatcher:

    '''
    generic dispatch method, calls "on_<ACTIONNAME>" if it is defined
    in this file, or returns a default error message.
    '''
    def _dispatch(self, r):
        if 'action' in r:
            method = r['action']
            try:
                method_to_call = getattr(self, 'on_%s' % method)
                logging.info('dispatch to method on_%s' % method)
            except AttributeError:
                logging.warn('cannot dispatch method %s' % method)
                return "Sorry I can't do this yet."
            return method_to_call(r)

    '''
    The default process method called on any request. Does make sure
    we behave like a proper DialogFlow fulfilment.
    '''
    def process(self):
        req = web.input()
        data = web.data()
        logging.info(req)
        try:
            d = loads(data)
            logging.info(pformat(d))
            if 'sessionId' in d:
                session_manager.set(d['sessionId'], d)

            r = self._dispatch(d['result'])
        except Exception as e:
            logging.warn("couldn't dispatch")
            return web.BadRequest("couldn't dispatch: %s" % e)
        web.header('Content-Type', 'application/json')

        response = {
          "speech": r,
          "displayText": r,
          "data": {},
          "contextOut": [],
          "source": "server"
        }

        return dumps(response)


class ROSDispatcher(FulfilmentDispatcher):
    '''
    goto action, expects argument "destination" referring to an
    existing topological node name in the robot's map
    '''
    def on_goto(self, d):
        node = d['parameters']['destination']
        logging.debug('called goto %s' % node)
        if ros:
            ros.goto_action_server.wait_for_server()
            goal = GotoNode()
            goal.target = node
            ros.goto_action_server.send_goal(goal)
            ros.goto_action_server.wait_for_result(
                rospy.Duration.from_sec(60.0)
            )
        return "I'm going to %s" % node

    '''
    speak action, expects argument "utterance" referring to an
    text that should be verbalised via Mary
    '''
    def on_speak(self, d):
        utterance = d['parameters']['utterance']
        logging.debug('called speak %s' % utterance)
        if ros:
            ros.speak_action_server.wait_for_server()
            goal = marytts()
            goal.text = utterance
            ros.speak_action_server.send_goal(goal)
            ros.speak_action_server.wait_for_result(
                rospy.Duration.from_sec(60.0)
            )
        return "I just said %s to the users." % utterance


class SimulationDispatcher(FulfilmentDispatcher):
    '''
    goto action, expects argument "destination" referring to an
    existing topological node name in the robot's map
    '''
    def on_goto(self, d):
        node = d['parameters']['destination']
        logging.debug('called goto %s' % node)
        simulation.set_location(self.robot, node)
        return "I'm going to %s" % node

    '''
    speak action, expects argument "utterance" referring to an
    text that should be verbalised via Mary
    '''
    def on_speak(self, d):
        utterance = d['parameters']['utterance']
        logging.debug('called speak %s' % utterance)
        simulation.add_utterance(self.robot, utterance)
        return "I just said %s to the users." % utterance


class index(SimulationDispatcher):
    def POST(self, robot):
        self.robot = robot
        return self.process()

    def GET(self, robot):
        self.robot = robot
        return self.process()


class test:

    def GET(self, robot):
        self.robot = robot
        logging.info('view %s' % robot)
        html = (
            '<head> <meta http-equiv="refresh" content="5" /> </head>'
            '<body>'
            '<h1>%s</h1>'
            '<table>'
            '<tr>'
            '  <td>Location:</td><td>%s</td>'
            '</tr>'
            '<tr>'
            '  <td>Last Utterance:</td><td>%s</td>'
            '</tr>' % (
                robot,
                simulation.get_location(robot),
                simulation.get_utterances(robot)[-1:]
              )
            )
        return html


def signal_handler(signum, frame):
    print "stopped."
    _exit(signal.SIGTERM)

app = web.application(urls, globals())

if __name__ == "__main__":
    app.internalerror = web.debugerror
    signal.signal(signal.SIGINT, signal_handler)
    app.run()
else:
    application = app.wsgifunc()
