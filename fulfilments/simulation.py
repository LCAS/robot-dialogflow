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
        return [a[3:] for a in dir(SimulationDispatcher)
                if a.startswith('on_')]

    def __init__(self):
        self.simulation = SimulationSingleton.getInstance()
        self.context = {}

    '''
    goto action, expects argument "destination" referring to an
    existing topological node name in the robot's map
    '''
    def on_goto(self, d):
        node = d['parameters']['destination']
        self.simulation.set(self.robot, 'last_action', 'goto("%s")' % node)
        logging.debug('called goto %s' % node)
        self.simulation.set(self.robot, 'location', node)
        self.context = self.simulation.state[self.robot]
        return "I'm going to %s" % node

    '''
    about a specific node in the map
    '''
    def on_about(self, d):
        node = d['parameters']['destination']
        self.simulation.set(self.robot, 'last_action', 'about("%s")' % node)
        logging.debug('called about for node %s' % node)
        self.context = self.simulation.state[self.robot]
        if node in TOPO_NODES:
            return "%s" % TOPO_NODES[node]['description']
        else:
            return "I don't know anything about %s" % node

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

    '''
    website action
    '''
    def on_website(self, d):
        url = d['parameters']['url']
        self.simulation.set(self.robot, 'last_action',
                            'website("%s")' % url)
        logging.debug('called website %s' % url)
        self.simulation.set(self.robot, 'website', url)
        self.context = self.simulation.state[self.robot]
        return "I show some information on my screen."

    '''
    wikipedia action
    '''
    def on_wikipedia(self, d):
        query = d['parameters']['query']
        self.simulation.set(self.robot, 'last_action',
                            'wikipedia("%s")' % query)
        logging.debug('called wikipedia %s' % query)
        return Wikipedia().query(query)

    '''
    eye's action
    '''
    def on_close_eyes(self, d):
        logging.debug('called close_eyes')
        self.simulation.set(self.robot, 'last_action',
                            'close_eyes()')
        self.simulation.set(self.robot, 'eyes_closed', True)
        self.context = self.simulation.state[self.robot]
        return "I closed my eyes"

    '''
    eye's action
    '''
    def on_open_eyes(self, d):
        logging.debug('called open_eyes')
        self.simulation.set(self.robot, 'last_action',
                            'close_eyes()')
        self.simulation.set(self.robot, 'eyes_closed', False)
        self.context = self.simulation.state[self.robot]
        return "I opened my eyes"

