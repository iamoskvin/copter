#!/usr/bin/env python

import socket

def checkInternetSocket(host="8.8.8.8", port=53, timeout=3):
    try:
        socket.setdefaulttimeout(timeout)
        socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, port))
        return True
    except socket.error as ex:
        print(ex)
        return False

if __name__ == '__main__':
    try:
        checkInternetSocket()
    except socket.error:
        pass