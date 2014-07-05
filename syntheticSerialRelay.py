from arduinoCalibrate import Calibration
import messagePublisher
import settings
from filters import *
from klampt import vectorops
import math
import time
import json
import sys

print """syntheticSerialRelay.py: sends wiggling messages to the
   Klamp't objective function server on localhost:3456.  Uses the calibration
   in arduinoCalibrate.txt to map to Klamp't configurations.  Sends a "config"
   type objective.
"""

calibration = Calibration()
calibration.load()

print "***** Using a deadband filter with exponential smoothing *****"
filter = CompositeFilter([DeadbandFilter(),ExponentialFilter(0.5)])

print "***** Running synthetic Arduino stream... *****"

def get_reading():
    #mags = [512,512,512,512,512]
    mags = [256,256,256,256,256]
    periods = [20,13,7,6,22]
    return [512 + mag*math.sin(time.time()/period) for mag,period in zip(mags,periods)]

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
        #map from arduino to Klamp'
        q = settings.arduinoToKlampt(q)
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
    publisher.closeServer()

