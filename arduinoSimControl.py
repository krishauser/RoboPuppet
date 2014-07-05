"""Controls a Klamp't simulation with the Arduino"""

from readArduino import *
from arduinoCalibrate import Calibration
from filters import *
from collections import defaultdict
import settings
from klampt import *
from klampt import vectorops
from klampt.glprogram import *
import time

synthetic = False
record = False
recordfn = 'arduinoRecord.csv'

if record:
    recordfile = open(recordfn,'w')
    recordfile.write('time,reading1,reading2,reading3,reading4,reading5,filtered1,filtered2,filtered3,filtered4,filtered5,sim1,sim2,sim3,sim4,sim5\n')

class GLControlGUI(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"Arduino control")
        self.world = world
        self.sim = Simulator(world)
        self.robot = world.robot(0)
        self.controller = self.sim.getController(0)

        if not synthetic:
            self.readthread = ReadArduinoThread()
            self.readthread.start()
            self.readthread.wait_for_data()
        self.calibration = Calibration()
        self.calibration.load()

        self.filter = CompositeFilter([DeadbandFilter(),ExponentialFilter(0.5)])
        self.last_reading = None
        self.qfiltered = None
        #null filter
        #self.filter = ExponentialFilter(1)

    def get_reading(self):
        if synthetic:
            return [1024,(time.time()*1000)%1024,1024,1024,1024]
        (cnt,timestamp,values)=self.readthread.get_data()
        t = time.time()
        #if t - timestamp > 0.05:
        print "Read thread delay",t-timestamp
        return [float(v) for v in values]

    def filter_reading(self,reading):
        return self.filter.process(reading)

    def update_input(self):
        self.raw_reading = self.get_reading()
        """
        if self.last_reading:
            if any(abs(b-a)>2 for (a,b) in zip(self.last_reading,self.raw_reading)):
                print "New reading..."
                self.last_reading = self.raw_reading
        else:
            self.last_reading = self.raw_reading
        """
        self.filtered_reading = self.filter_reading(self.raw_reading)
        q = self.calibration.readingToConfig(self.filtered_reading)
        #map to Klamp't configs
        q = settings.arduinoToKlampt(q)
        qmin,qmax = self.robot.getJointLimits()
        for i,(qi,a,b) in enumerate(zip(q,qmin,qmax)):
            buf = 1e-7*(b-a)
            q[i] = min(b-buf,max(qi,a+buf))
        """
        if self.qfiltered!=None:
            if any(abs(a-b)>1e-4 for (a,b) in zip(q,self.qfiltered)):
                print "New input"
        """
        self.qfiltered = q
        self.controller.setMilestone(q)

    def display(self):
        self.sim.updateWorld()
        self.world.drawGL()
        
        #draw commanded configurations
        if self.qfiltered!=None:
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
            glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,0.5,0,0.5])
            self.world.robot(0).setConfig(self.qfiltered)
            self.world.robot(0).drawGL(False)
            glDisable(GL_BLEND)


    def idle(self):
        if not hasattr(self,'iteration'):
            self.iteration=0
        if not hasattr(self,'tlast'):
            self.tlast=time.time()
        self.iteration += 1
        if self.iteration % 30 == 0:
            t = time.time()
            print "Simulation time: %f, clock time: %g\n"%(self.dt*30,t-self.tlast)
            self.tlast = t
        self.update_input()
            
        self.sim.simulate(self.dt)
        if record:
            recordfile.write('%f,'%(self.sim.getTime(),))
            recordfile.write(','.join(str(x) for x in self.calibration.readingToConfig(self.raw_reading)))
            recordfile.write(',')
            recordfile.write(','.join(str(x) for x in self.calibration.readingToConfig(self.filtered_reading)))
            recordfile.write(',')
            recordfile.write(','.join(str(x) for x in self.controller.getCommandedConfig()[1:6]))
            recordfile.write('\n')
        glutPostRedisplay()


if __name__ == '__main__':
    world = WorldModel()
    if not world.readFile(settings.simControlWorldFile):
        raise IOError("Unable to read file")
    gui = GLControlGUI(world)
    gui.run()

