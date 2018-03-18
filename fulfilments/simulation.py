from collections import defaultdict
import logging
from fulfilment import FulfilmentDispatcher
from utils import Wikipedia


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

        self.state = defaultdict(lambda: {
            'location': '*unknown*',
            'utterances': [],
            'eyes_closed': False,
            'track_people': False
            })

    def get(self, robot, attribute='location'):
        return self.state[robot][attribute]

    def set(self, robot, attribute='location', value=None):
        self.state[robot][attribute] = value

    def append(self, robot, attribute='location', value=None):
        self.state[robot][attribute].append(value)


class SimulationDispatcher(FulfilmentDispatcher):

    def __init__(self):
        self.simulation = SimulationSingleton.getInstance()
        self.context = {}

    '''
    goto action, expects argument "destination" referring to an
    existing topological node name in the robot's map
    '''
    def on_goto(self, d):
        node = d['parameters']['destination']
        logging.debug('called goto %s' % node)
        self.simulation.set(self.robot, 'location', node)
        self.context = self.simulation.state[self.robot]
        return "I'm going to %s" % node

    '''
    speak action, expects argument "utterance" referring to an
    text that should be verbalised via Mary
    '''
    def on_speak(self, d):
        utterance = d['parameters']['utterance']
        logging.debug('called speak %s' % utterance)
        self.simulation.append(self.robot, 'utterances', utterance)
        self.context = self.simulation.state[self.robot]
        return "I just said %s to the users." % utterance

    '''
    speak action, expects argument "utterance" referring to an
    text that should be verbalised via Mary
    '''
    def on_wikipedia(self, d):
        query = d['parameters']['query']
        logging.debug('called wikipedia %s' % query)
        return Wikipedia().query(query)
