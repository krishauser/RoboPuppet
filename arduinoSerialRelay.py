from readArduino import *
from arduinoCalibrate import Calibration
import messagePublisher
import settings
from filters import *
from klampt import vectorops
import time
import json
import sys

print """ardunoSerialRelay.py: relays messages from an Arduino reader to a
   Klamp't objective function server on localhost:3456.  Uses the calibration
   in arduinoCalibrate.txt to map to Klamp't configurations.  Sends a "config"
   type objective.

Options:
   -synthetic: simulates the Arduino data (moves robot in circle)
   -port=fn: sets the Arduino read port (default %s)

"""%(settings.defaultPort,)

port = settings.defaultPort
synthetic = False
for arg in sys.argv[1:]:
    if arg=="-synthetic":
        synthetic = True
    elif arg.startswith("-port="):
        port = arg[6:]

calibration = Calibration()
calibration.load()

print "***** Using a deadband filter with exponential smoothing *****"
filter = CompositeFilter([DeadbandFilter(),ExponentialFilter(0.5)])

if not synthetic:
    print "***** Opening Arduino reader from stream %s *****"%(port,)
    readthread = ReaderThread(ReadArduinoProcess(port))
    readthread.start()
    readthread.wait_for_data()
    print "***** Successfully started Arduino reader... *****"
else:
    print "***** Running synthetic Arduino stream... *****"

def get_reading():
    if synthetic:
        return [512,(time.time()*100)%1024,512,512,512]
    (cnt,timestamp,values)=readthread.get_data()
    return [float(v) for v in values]

rate = 30
host = ''
port = 3456
backlog = 1
print "***** Setting up socket server on port %d. *****"%(port,)
publisher = messagePublisher.Publisher((host,port),backlog)
print "***** Waiting for clients... *****"""
publisher.accept()

try:
    print "***** Streaming data at %sHz... *****"%(str(rate),)
    while True:
        reading = filter.process(get_reading())
        q = calibration.readingToConfig(reading)
        #Convert to Klamp't configs
        q = [0]+q+[0]*(7-1-len(q))
        msg = {"type":"config","data":q}
        msgstr = json.dumps(msg)
        #print msgstr,'length',len(msgstr)
        try:
            publisher.write(msgstr)
        except IOError:
            print "***** Client error, killing connection. *****"
            publisher.closeClient()
            print "***** Waiting for clients... *****"
            publisher.accept()
            print "***** Streaming data at %sHz... *****"%(str(rate),)
        time.sleep(1.0/rate)
except KeyboardInterrupt,SystemExit:
    if not synthetic:
        readthread.quit()
    publisher.closeServer()

