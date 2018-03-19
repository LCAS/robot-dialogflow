import logging
from fulfilment import FulfilmentDispatcher


class ROSInterface(object):
    _instance = None

    def __new__(cls, *args, **kwargs):
        """
        make it a singleton
        """
        if not cls._instance:
            cls._instance = super(ROSInterface, cls).__new__(
                                cls, *args, **kwargs)
        return cls._instance

    def __init__(self):
        try:
            import rospy
            from actionlib import SimpleActionClient
            from topological_navigation.msg import GotoNode
            from mary_tts.msg import marytts
            self.ros_available = True
            logging.info('running in ROS mode')
            rospy.init_node('dialogflow_fulfilment')
            self.goto_action_server = SimpleActionClient(
                '/topological_navigation',
                GotoNode)
            self.speak_action_server = SimpleActionClient(
                '/speak',
                marytts)
        except:
            self.ros_available = False
            logging.info('running without ROS')


class ROSDispatcher(FulfilmentDispatcher):

    def __init__(self):
        super(ROSDispatcher, self).__init__(world=None, robot=None)

    '''
    goto action, expects argument "destination" referring to an
    existing topological node name in the robot's map
    '''
    def on_goto(self, d):
        node = d['parameters']['destination']
        logging.debug('called goto %s' % node)
        if ROSInterface().ros_available:
            ROSInterface().goto_action_server.wait_for_server()
            goal = GotoNode()
            goal.target = node
            ROSInterface().goto_action_server.send_goal(goal)
            ROSInterface().goto_action_server.wait_for_result(
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
