"""A socket that publishes variable-length messages such that the first 4
bytes are the length of the message (in binary) and the remainder is the
payload.  Used to send messages to Klamp't."""

import socket
import threading
import time

headerlen = 4

def packStrlen(s):
    l = len(s)
    assert(l <= 0xffffffff)
    bytes = [None]*4
    bytes[0] = chr(l&0xff)
    bytes[1] = chr((l>>8)&0xff)
    bytes[2] = chr((l>>16)&0xff)
    bytes[3] = chr((l>>24)&0xff)
    return ''.join(bytes)

def unpackStrlen(s):
    assert len(s)==headerlen
    return (ord(s[3])<<24)|(ord(s[2])<<16)|(ord(s[1])<<8)|ord(s[0])

def writeSocket(socket,msg):
    totalsent = 0
    while totalsent < len(msg):
        sent = socket.send(msg[totalsent:])
        if sent == 0:
            raise IOError("socket connection broken")
        totalsent = totalsent + sent
    return

def readSocket(socket,length):
    chunk = socket.recv(length)
    msg = chunk
    while len(msg) < length:
        chunk = socket.recv(length-len(msg))
        if chunk == '':
            raise IOError("socket connection broken")
        msg = msg + chunk
    return msg

class Publisher:
    def __init__(self,addr,maxclients=1):
        #create an INET, STREAMing socket
        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversocket.bind(addr)
        #become a server socket
        self.serversocket.listen(maxclients)
        self.clientsockets = []
    def accept(self):
        """Accept connections from clients."""
        (sock, addr) = self.serversocket.accept()
        self.clientsockets.append((sock,addr))
        time.sleep(0.1)
        return len(self.clientsockets)-1
    
    def write(self,packet,client=0):
        """Write a packet to the given client, or all clients if
        client='all'.  Format is 4 byte length + packet
        """
        assert unpackStrlen(packStrlen(packet))==len(packet)
        msg = packStrlen(packet)+packet
        if client == 'all':
            for c in self.clientsockets:
                writeSocket(c[0],msg)
        else:
            writeSocket(self.clientsockets[client][0],msg)
            
    def read(self,client=0):
        """Reads a string from a client.  Blocking.  Format is
        4 byte length + packet"""
        s = self.clientsockets[client][0]
        print "reading header..."
        packlen = readSocket(s,headerlen)
        packlen = unpackStrlen(packlen)
        print "reading message of length",packlen,"..."
        return readSocket(s,packlen)

    def closeClient(self,client=0):
        if client == 'all':
            for c in self.clientsockets:
                c[0].close()
            self.clientsockets = []
        else:
            self.clientsockets[client][0].close()
            del self.clientsockets[client]
    def closeServer(self):
        for c in self.clientsockets:
            c[0].close()
        self.clientsockets = []
        self.serversocket.close()        

class Subscriber:
    def __init__(self,addr=('',1234)):
        self.addr = addr
        self.s = None
    def read(self,timeout=None):
        if timeout!=None:
            tstart = time.time()
        while self.s == None:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try: 
                self.s.connect(self.addr)
            except Exception as e:
                print e
                self.s.close()
                self.s = None
            if self.s == None and timeout!=None:
                if time.time()-tstart > timout:
                    raise RuntimeError("Socket couldn't connect")

        lengthstr = readSocket(self.s,headerlen)
        assert len(lengthstr)==headerlen
        length = unpackStrlen(lengthstr)
        return readSocket(self.s,length)

class MultiPublisher:
    def __init__(self,addr,maxclients=1):
        #create an INET, STREAMing socket
        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversocket.bind(addr)
        #become a server socket
        self.serversocket.listen(maxclients)
        self.clients = []
        self.do_stop = False
    def start():
        while not self.stop:
            #accept connections from outside
            (clientsocket, address) = self.serversocket.accept()
            #now do something with the clientsocket
            #in this case, we'll pretend this is a threaded server
            ct = self.makeWorker(clientsocket)
            ct.start()
            self.clients.append(ct)
        for c in self.clients:
            c.stop()
        for c in self.clients:
            c.join()
    def stop(self):
        self.do_stop = True
    def makeWorker(self):
        #return some worker thread
        raise NotImplementedError()

class PublisherThread(threading.Thread):
    def __init__(self,socket):
        thread.Thread.__init__(self)
        self.s=socket
        self.do_stop = False
    def stop():
        self.do_stop = True
    def run():
        """Subclasses should override this"""
        return
    def write(self,msg):
        msg = packStrlen(str)+msg
        writeSocket(self.s,msg)
        return

