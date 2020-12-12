#!/usr/bin/env python3

import tornado.ioloop
import tornado.web
from tornado.web import RequestHandler, StaticFileHandler
import socket
import struct
import time
import os

HOST = 'localhost'
PORT = 5678

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


class IndexHandler(RequestHandler):
    def get(self):
        self.redirect('/static/baidumap.html')


class PostHandler(RequestHandler):
    def post(self):
        self.set_header('Content-Type', 'text/plain')
        pos = [
            float(self.get_body_argument(k)) for k in ['lon', 'lat', 'hgt']
        ]
        data = struct.pack('ddd', *pos)
        sock.sendto(data, (HOST, PORT))
        print(*pos)
        self.write('OK')


def make_app():
    return tornado.web.Application([
        (r'/', IndexHandler),
        (r'/post', PostHandler),
        (r'/static/(.*)', StaticFileHandler, {
            'path': os.path.dirname(__file__)
        })
    ])


if __name__ == "__main__":
    from argparse import ArgumentParser
    argp = ArgumentParser(description='GPS-SDR-SIM realtime map server.')
    argp.add_argument('port',
                      type=int,
                      default=8080,
                      nargs='?',
                      help='specify port for map server [default: 8080]')
    argp.add_argument(
        '--host',
        type=str,
        default='0.0.0.0',
        help='specify host map server to bind [default: 0.0.0.0]')
    args = argp.parse_args()

    app = make_app()
    print('Serving at http://{}:{}'.format(args.host, args.port))
    app.listen(args.port, args.host)
    tornado.ioloop.IOLoop.current().start()
