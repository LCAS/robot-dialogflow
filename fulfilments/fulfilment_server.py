#!/usr/bin/env python
import web

import logging
from json import dumps
from os import _exit
import signal
from time import time

from simulation import SimulationDispatcher, SimulationSingleton

from agent import Agent

logging.basicConfig(level=logging.DEBUG)


urls = (
    '/webhook/(.+)', 'webhook',
    '/(.+)', 'index'
)


class webhook(SimulationDispatcher):
    def POST(self, robot):
        self.robot = robot
        return self.process()

    def GET(self, robot):
        self.robot = robot
        return self.process()


# class test:

#     def GET(self, robot):
#         self.robot = robot
#         logging.info('view %s' % robot)
#         html = (
#             '<head> <meta http-equiv="refresh" content="5" /> </head>'
#             '<body>'
#             '<h1>%s</h1>'
#             '<table>'
#             '<tr>'
#             '  <td>Location:</td><td>%s</td>'
#             '</tr>'
#             '<tr>'
#             '  <td>Last Utterance:</td><td>%s</td>'
#             '</tr>' % (
#                 robot,
#                 self.simulation.get_location(robot),
#                 self.simulation.get_utterances(robot)[-1:]
#               )
#             )
#         return html


class index:

    def GET(self, robot):
        self.robot = robot
        logging.info('status %s' % robot)

        response = {
          "state": SimulationSingleton.getInstance().state[robot],
          "robot": robot,
          "ts": (time()),
          # "wiki": Wikipedia().query('lincolnsdfdsf')
        }
        Agent().query('tell me about lincoln')
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
