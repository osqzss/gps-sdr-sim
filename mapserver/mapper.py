#!/usr/bin/env python3

from http.server import SimpleHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs
import socket
import struct
import time
import os

HOST = 'localhost'
PORT = 5678

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


class MapHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        directory = os.path.dirname(__file__)
        super().__init__(*args, directory=directory, **kwargs)

    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/static/baidumap.html')
            self.end_headers()
        else:
            super(MapHandler, self).do_GET()

    def do_POST(self):
        if self.path == '/post':
            urlencoded = self.rfile.read(int(self.headers['Content-Length']))
            parsed = parse_qs(urlencoded)
            pos = [float(parsed.get(k)[0]) for k in [b'lon', b'lat', b'hgt']]
            data = struct.pack('ddd', *pos)
            sock.sendto(data, (HOST, PORT))
            print(*pos)
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
        else:
            self.send_response(404)
        self.end_headers()


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

    httpd = HTTPServer((args.host, args.port), MapHandler)
    print('Serving at http://{}:{}'.format(args.host, args.port))
    httpd.serve_forever()
