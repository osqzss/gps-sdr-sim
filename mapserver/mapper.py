#!/usr/bin/env python
    #-*-coding:utf-8-*-
import socket
import struct
import time
import web
HOST = '127.0.0.1'    
PORT = 5678            
BUFFSIZE = 1024          
ADDR = ( HOST, PORT )   

sock = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )



urls = (
	'/','index',
	'/post','post',
	'/static/*','static',
)
app = web.application(urls, globals())

class index:
	def GET(self):
		return ""

class post:
	def POST(self):
		inn=web.input()
		lon=float(inn.get('lon'))
		lat=float(inn.get('lat'))
		h=float(inn.get('hgt'))
		data=struct.pack("ddd",lat,lon,h)
		sock.sendto(data,ADDR)
		print lon,lat,h

class static:
    def GET(self, media, file):
        try:
            f = open(media+'/'+file, 'r')
            return f.read()
        except:
            return '404'
		
if __name__ == "__main__": app.run()



