import logging
import web
from json import loads, dumps
from pprint import pformat


class FulfilmentDispatcher:

    def can_dispatch(self, method):
        try:
            getattr(self, 'on_%s' % method)
            return True
        except AttributeError:
            return False

    '''
    generic dispatch method, calls "on_<ACTIONNAME>" if it is defined
    in this file, or returns a default error message.
    '''
    def _dispatch(self, r):
        logging.info("called dispatch: %s" % r)
        if 'action' in r:
            method = r['action']
            try:
                method_to_call = getattr(self, 'on_%s' % method)
                logging.info('dispatch to method on_%s' % method)
            except AttributeError:
                logging.warn('cannot dispatch method %s' % method)
                return ""
            return method_to_call(r)

    '''
    The default process method called on any request. Does make sure
    we behave like a proper DialogFlow fulfilment.
    '''
    def process(self):
        req = web.input()
        data = web.data()
        logging.info(req)
        self.context = {}
        try:
            d = loads(data)
            response = self.dispatch(d)
            web.header('Content-Type', 'application/json')
            return dumps(response)
        except Exception as e:
            logging.warn("couldn't dispatch")
            return web.BadRequest("couldn't dispatch: %s" % e)

    def dispatch(self, d):
        r = self._dispatch(d['result'])

        response = {
          "speech": r,
          "displayText": r,
          "data": {},
          "contextOut": [
              {
                  "name": "robot_state",
                  "parameters": self.context,
                  "lifespan": 5
              }],
          "source": "server"
        }
        return response
