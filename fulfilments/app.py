#!/usr/bin/env python
import web
import sys
import logging
from json import dumps, loads
from os import _exit, chdir, path
import signal
from time import time
from collections import defaultdict
from uuid import uuid4
from mimetypes import guess_type
abspath = path.dirname(__file__)
print abspath

if len(abspath) > 0:
    sys.path.append(abspath)
    chdir(abspath)

from simulation import SimulationDispatcher, SimulationSingleton

from agent import Agent
from topomap import TOPO_NODES

logging.basicConfig(level=logging.DEBUG)


urls = (
    '/(.+)/webhook', 'webhook',
    '/api/(.+)', 'api',
    '/assets/(.+)', 'assets',
    '/(.+)', 'index',
    '/', 'redirect'
)


class assets:
    def GET(self, file):
        try:
            f = open(abspath + '/assets/' + file, 'r')
            (t, encoding) = guess_type(file)
            print t
            web.header("Content-Type", t)
            return f.read()
        except Exception as e:
            print e, file
            return web.notfound()


class redirect:

    def GET(self):
        web.seeother('https://github.com/LCAS/robot-dialogflow/wiki')


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
        session = web.cookies(df_session=uuid4()).df_session
        web.setcookie('df_session', session, 3600)

        methods = SimulationDispatcher.available_methods()
        return index.render.index(robot, TOPO_NODES, methods)

    def POST(self, robot):
        session = web.cookies(df_session=uuid4()).df_session
        web.setcookie('df_session', session, 3600)
        wi = web.input(
            query="who are you?",
            input="text",
            event="",
            action_params="",
            parameters="{}")
        agent = index.agents[robot]
        # use robot identifier also as API key
        if wi.input == 'text':
            response = agent.query(
                wi.query, session=session, robot=robot, apikey=robot
                )
        elif wi.input == 'action':
            sd = SimulationDispatcher()
            sd.robot = robot
            sd.dispatch(loads(wi.action_params))
            response = ''
        elif wi.input == 'event':
            logging.info(wi.parameters)
            response = agent.send_event(
                wi.event, loads(wi.parameters),
                session=session, robot=robot, apikey=robot
                )
        else:
            return web.nomethod()
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
