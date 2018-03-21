from collections import defaultdict
import logging
from fulfilment import FulfilmentDispatcher
from utils import Wikipedia
from threading import Condition
from topomap import TOPO_NODES


class SimulationSingleton:
    # Here will be the instance stored.
    __instance = None

    @staticmethod
    def getInstance():
        """ Static access method. """
        if SimulationSingleton.__instance is None:
            SimulationSingleton()
        return SimulationSingleton.__instance

    def __init__(self):
        """ Virtually private constructor. """
        if SimulationSingleton.__instance is not None:
            raise Exception("This class is a singleton!")
        else:
            SimulationSingleton.__instance = self

        self.update_cond = Condition()

        self.state = defaultdict(lambda: {
            'location': '*unknown*',
            'utterances': [],
            'website': 'https://lcas.lincoln.ac.uk/wp/',
            'eyes_closed': False,
            'track_people': False
            })

    def get(self, robot, attribute='location'):
        return self.state[robot][attribute]

    def set(self, robot, attribute='location', value=None):
        self.update_cond.acquire()
        self.state[robot][attribute] = value
        try:
            self.update_cond.notifyAll()
        finally:
            self.update_cond.release()

    def append(self, robot, attribute='location', value=None):
        self.update_cond.acquire()
        self.state[robot][attribute].append(value)
        try:
            self.update_cond.notifyAll()
        finally:
            self.update_cond.release()


class SimulationDispatcher(FulfilmentDispatcher):

    @staticmethod
    def available_methods():
        methods = [
            a[3:] for a in dir(SimulationDispatcher)
            if a.startswith('on_')
            ]
        result = {}
        for m in methods:
            result[m] = SimulationDispatcher.get_doc(m)
        return result

    @staticmethod
    def get_doc(m):
        method = getattr(SimulationDispatcher, 'on_%s' % m)
        return method.__doc__

    def __init__(self):
        self.simulation = SimulationSingleton.getInstance()
        self.context = {}

    def on_goto(self, d):
        '''
        Moves the robot to a specified node in the robot's map.<br>
        Requires one parameter "<tt>destination</tt>"" referring to an
        existing topological node name in the robot's map.
        The robot will reply with <i>'I'm going to <tt>destination</tt>'</i>
        if it can go there or with '<i>I don't know where
        <tt>destination</tt> is'</i> if it cannot find it in its map.
        '''
        node = d['parameters']['destination']
        self.simulation.set(self.robot, 'last_action', 'goto("%s")' % node)
        if node in TOPO_NODES:
            logging.debug('called goto %s' % node)
            self.simulation.set(self.robot, 'location', node)
            self.context = self.simulation.state[self.robot]
            return "I'm going to %s" % node
        else:
            return "I don't know where %s is." % node

    def on_about(self, d):
        '''
        Returns the description for a specific node in the robot's map.<br>
        Requires one parameter "<tt>destination</tt>" referring to an
        existing topological node name in the robot's map.
        '''
        node = d['parameters']['destination']
        self.simulation.set(self.robot, 'last_action', 'about("%s")' % node)
        logging.debug('called about for node %s' % node)
        self.context = self.simulation.state[self.robot]
        if node in TOPO_NODES:
            return "%s" % TOPO_NODES[node]['description']
        else:
            return "I don't know anything about %s" % node

    def on_whereami(self, d):
        '''
        Returns the current position of the robot as an utterance like
        <i>"I'm at a place called 'boat'"</i>.<br>
        It doesn't require any arguments.
        '''
        self.simulation.set(self.robot, 'last_action', 'whereami()')
        logging.debug('called whereami for node')
        node = self.simulation.state[self.robot]['location']
        return "I'm at a place called %s" % node

    '''
    speak action, expects argument "utterance" referring to an
    text that should be verbalised via Mary
    '''
    # def on_speak(self, d):
    #     utterance = d['parameters']['utterance']
    #     self.simulation.set(self.robot, 'last_action',
    #                         'speak("%s")' % utterance)
    #     logging.debug('called speak %s' % utterance)
    #     self.simulation.append(self.robot, 'utterances', utterance)
    #     self.context = self.simulation.state[self.robot]
    #     return "I just said %s to the users." % utterance

    def on_website(self, d):
        '''
        Displays a specified website on the robot's screen.<br>
        Requires one parameter "<tt>url</tt>" referring to the website,
        e.g. <tt>https://lcas.lincoln.ac.uk/wp/</tt><br>
        <u>Note:</u> Some websites don't allow to be displayed within another
        website, so it can result in the page not being shown.
        '''
        url = d['parameters']['url']
        self.simulation.set(self.robot, 'last_action',
                            'website("%s")' % url)
        logging.debug('called website %s' % url)
        self.simulation.set(self.robot, 'website', url)
        self.context = self.simulation.state[self.robot]
        return "I show some information on my screen."

    def on_wikipedia(self, d):
        '''
        Search for a specific query on wikipedia.<br>
        Requires one parameter "<tt>query</tt>" referring to
        the phrase searched for.<br>
        This action returns the short description of the best matched
        for the given query. Depending on the query, it is possible that
        an answer is not found.
        '''
        query = d['parameters']['query']
        self.simulation.set(self.robot, 'last_action',
                            'wikipedia("%s")' % query)
        logging.debug('called wikipedia %s' % query)
        return Wikipedia().query(query)

    def on_close_eyes(self, d):
        '''
        Closes the robot's eyes. It does not require any parameters.
        '''
        logging.debug('called close_eyes')
        self.simulation.set(self.robot, 'last_action',
                            'close_eyes()')
        self.simulation.set(self.robot, 'eyes_closed', True)
        self.context = self.simulation.state[self.robot]
        return "I closed my eyes"

    def on_toggle_eyes(self, d):
        '''
        Closes the robot's eyes if they were open and opens them
        if they were closed. It does not require any parameters.
        '''
        logging.debug('called toggle_eyes')
        self.simulation.set(self.robot, 'last_action',
                            'toggle_eyes()')
        self.simulation.set(
            self.robot, 'eyes_closed',
            not self.simulation.get(self.robot, 'eyes_closed')
            )
        self.context = self.simulation.state[self.robot]
        return "I closed my eyes"

    def on_open_eyes(self, d):
        '''
        Opens the robot's eyes. It does not require any parameters.
        '''
        logging.debug('called open_eyes')
        self.simulation.set(self.robot, 'last_action',
                            'open_eyes()')
        self.simulation.set(self.robot, 'eyes_closed', False)
        self.context = self.simulation.state[self.robot]
        return "I opened my eyes"

