#!/usr/bin/env python
import web
import sys
import logging
from json import dumps
from os import _exit, chdir, path
import signal
from time import time
from collections import defaultdict


abspath = path.dirname(__file__)
print abspath

if len(abspath) > 0:
    sys.path.append(abspath)
    chdir(abspath)


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

    def response(self, data):
        response = "data: " + data + "\n\n"
        return response

    def GET(self, robot):
        self.robot = robot
        logging.info('status %s' % robot)

        block = False
        web.header("Content-Type", "text/event-stream")
        web.header('Cache-Control', 'no-cache')

        while True:
            SimulationSingleton.getInstance().update_cond.acquire()
            try:
                if block:
                    SimulationSingleton.getInstance().update_cond.wait(
                        timeout=60
                        )
                    #logging.info('wait returned, yielding new result')
            finally:
                SimulationSingleton.getInstance().update_cond.release()
            block = True

            data = {
              "state": SimulationSingleton.getInstance().state[robot],
              "robot": robot,
              "ts": (time()),
              "dummy": range(1000)
              # "wiki": Wikipedia().query('lincolnsdfdsf')
            }
            r = self.response(dumps(data))
            yield r

        # response = {
        #   "state": SimulationSingleton.getInstance().state[robot],
        #   "robot": robot,
        #   "ts": (time()),
        #   # "wiki": Wikipedia().query('lincolnsdfdsf')
        # }
        # #Agent().query('tell me about lincoln')
        # web.header('Content-Type', 'application/json')
        # return dumps(response)


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
