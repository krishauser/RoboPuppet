"""Threads that read serial input from the arduino.  Can either poll
or wait on the data, see the rundemo function for more details"""

import serial
import time
from threading import Lock, Thread, Condition
import settings


class ReadArduinoProcess:
    def __init__(self,port=settings.defaultPort,baud=9600):
        self.ser = serial.Serial(port,baud)
    def start(self):
        self.firstTime = True
    def do(self):
        while True:
            try:
                rawData = self.ser.readline()
            except SerialException:
                print "Error reading from serial port"
                continue
            if self.firstTime:
                #skip the first line
                while len(rawData.strip())==0:
                    rawData = self.ser.readline()
                self.firstTime = False
            while len(rawData.strip())==0:
                rawData = self.ser.readline()
            return rawData.split()
        return
    def stop(self):
        self.ser.close()

class ReaderThread(Thread):
    def __init__(self,process):
        Thread.__init__(self)
        self.daemon = True
        self.process = process
        self.lock = Lock()
        self.condition = Condition(self.lock)
        self.values = []
        self.new_data_ready = False
        self.data_timestamp = None
        self.data_count = 0
        self.do_quit = False

    def wait_for_data(self):
        """Calling thread: waits until some un-read data is ready."""
        if not self.new_data_ready:
            self.wait_for_new_data()

    def wait_for_new_data(self):
        """Calling thread: waits until some un-read data is ready.
        Only difference between this and wait_for_data is that this waits
        until the next datum that arrives after the call of wait_for_new_data,
        while wait_for_data waits for the next datum after the prior get_data
        call."""
        self.condition.acquire()
        self.condition.wait()
        self.condition.release()

    def is_new_data_ready(self):
        """True if a new piece of data arrived after the last get_data call."""
        return self.new_data_ready

    def quit(self):
        """Tells the reading thread to exit."""
        self.do_quit = True

    def get_data(self):
        """Calling thread: returns a tuple containing the ID of the message,
        its timestamp, and the payload values.
        
        This also marks that the latest data have been read.
        """
        res = None
        with self.lock:
            self.new_data_ready = False
            res=(self.data_count,self.data_timestamp,self.values)
        return res
        
    def run(self):
        self.process.start()
        self.tlast = time.time()
        while not self.do_quit:
            try:
                values = self.process.do()
            except KeyboardInterrupt,SystemExit:
                break
    
            with self.lock:
                self.new_data_ready = True
                self.data_count += 1
                self.data_timestamp = time.time()
                self.values = values
                self.condition.notify()
            
            if self.data_count % 30 == 0:
                t = time.time()
                print "avg read time",(t-self.tlast)/30
                self.tlast = t
        self.process.stop()

def ReadArduinoThread(port=settings.defaultPort):
    return ReaderThread(ReadArduinoProcess(port))

def rundemo(maxCount=100):
    polling_demo = True
    readthread = ReadArduinoThread()
    readthread.start()
    starttime = time.time()
    try:
        if polling_demo:
            #do polling on thread
            while True:
                if readthread.is_new_data_ready():
                    (cnt,timestamp,values)=readthread.get_data()
                    print "Message",cnt,"time",timestamp-starttime,"values",values
                    if cnt > maxCount:
                        readthread.quit()
                        break
                #sleep for 1 ms
                time.sleep(0.001)
        else:
            #catch signals from thread
            while True:
                readthread.wait_for_data()
                (cnt,timestamp,values)=readthread.get_data()
                print "Message",cnt,"time",timestamp-starttime,"values",values
                if cnt > maxCount:
                    readthread.quit()
                    break
    except (KeyboardInterrupt,SystemExit):
        readthread.quit()
    print "Joining terminated thread..."
    readthread.join()
    print "Done."

if __name__ == '__main__':
    rundemo()    
