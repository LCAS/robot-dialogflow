#!/usr/bin/env python
import web

import logging
from json import dumps
from os import _exit
import signal
from time import time
from collections import defaultdict

from simulation import SimulationDispatcher, SimulationSingleton

from agent import Agent

logging.basicConfig(level=logging.DEBUG)


urls = (
    '/webhook/(.+)', 'webhook',
    '/api/(.+)', 'api',
    '/(.+)', 'index'
)


class webhook(SimulationDispatcher):
    def POST(self, robot):
        self.robot = robot
        return self.process()

    def GET(self, robot):
        self.robot = robot
        return self.process()


class api:

    def GET(self, robot):
        self.robot = robot
        logging.info('status %s' % robot)

        response = {
          "state": SimulationSingleton.getInstance().state[robot],
          "robot": robot,
          "ts": (time()),
          # "wiki": Wikipedia().query('lincolnsdfdsf')
        }
        #Agent().query('tell me about lincoln')
        web.header('Content-Type', 'application/json')
        return dumps(response)


class index:
    render = web.template.render('templates/', base='base')
    simulation = SimulationSingleton.getInstance()
    agents = defaultdict(Agent)

    def GET(self, robot):
        return index.render.index(robot)

    def POST(self, robot):
        wi = web.input(query="who are you?")
        response = index.agents[robot].query(
            wi.query, robot
            )
        web.header('Content-Type', 'application/json')
        return dumps(response)


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
