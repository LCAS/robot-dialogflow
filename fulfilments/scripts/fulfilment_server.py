#!/usr/bin/env python
import web

import logging
from json import loads, dumps
from pprint import pformat

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
    '/webhook', 'index'
)


class ROSInterface:

    def __init__(self):
        rospy.init_node('dialogflow_fulfilment')
        self.goto_action_server = SimpleActionClient(
            '/topological_navigation',
            GotoNode)
        self.speak_action_server = SimpleActionClient(
            '/speak',
            marytts)

if ros_available:
    ros = ROSInterface()
else:
    ros = None


class index:

    def POST(self):
        return self.process()

    def GET(self):
        return self.process()

    def on_add_item(self, d):
        logging.debug('called add_item')
        return "it truly worked"

    '''
    goto action, excepts argument "destination" referring to an
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
    speak action, excepts argument "utterance" referring to an
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
        d = loads(data)
        logging.info(pformat(d))
        try:
            r = self._dispatch(d['result'])
        except Exception as e:
            logging.warn("couldn't dispatch")
            return web.BadRequest(e)
        web.header('Content-Type', 'application/json')

        response = {
          "speech": r,
          "displayText": r,
          "data": {},
          "contextOut": [],
          "source": "server"
        }

        return dumps(response)
    # action = req.get('result').get('action')

    # # Check if the request is for the translate action
    # if action == 'translate.text':
    #     # Get the parameters for the translation
    #     text = req['result']['parameters'].get('text')
    #     source_lang = req['result']['parameters'].get('lang-from')
    #     target_lang = req['result']['parameters'].get('lang-to')

    #     # Fulfill the translation and get a response
    #     output = translate(text, source_lang, target_lang)

    #     # Compose the response to API.AI
    #     res = {'speech': output,
    #            'displayText': output,
    #            'contextOut': req['result']['contexts']}
    # else:
    #     # If the request is not to the translate.text action throw an error
    #     LOG.error('Unexpected action requested: %s', json.dumps(req))
    #     res = {'speech': 'error', 'displayText': 'error'}

    # return make_response(jsonify(res))


if __name__ == "__main__":

    app = web.application(urls, globals())
    app.internalerror = web.debugerror
    app.run()
