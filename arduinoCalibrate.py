"""Perform a visual calibration given the configurations in calibrate.configs.
Saves the result to arduinoCalibrate.txt."""

from readArduino import *
import settings
from klampt import *
from klampt import vectorops
from klampt.glprogram import *
from klampt.loader import readVector

def inv2x2(A):
    det = A[0][0]*A[1][1]-A[0][1]*A[1][0]
    scale = 1.0/det
    return [[scale*A[1][1],-scale*A[0][1]],
            [-scale*A[1][0],scale*A[0][0]]]

def loadCalibrateConfigs(fn):
    """Loads a .configs.file into a list of configurations"""
    f = open(fn,'r')
    res = []
    for line in f.readlines():
        res.append(readVector(line))
    return res

class Calibration:
    """A linear calibration q[i] = a[i]v[i]+b[i]
    """
    def __init__(self,numValues=0):
        self.coeffs = [1]*numValues
        self.offsets = [0]*numValues
        return
    def readingToConfig(self,readings):
        if len(readings)!=len(self.coeffs):
            raise ValueError("Incorrect size of readings (%d, should be %d)"%(len(readings),len(self.coeffs)))
        return [ai*vi+bi for (ai,bi,vi) in zip(self.coeffs,self.offsets,readings)]
    def configToReading(self,config):
        if len(config)!=len(self.coeffs):
            raise ValueError("Incorrect size of config (%d, should be %d)"%(len(readings),len(self.coeffs)))
        return [(qi-bi)/ai for (ai,bi,vi) in zip(self.coeffs,self.offsets,config)]
    def calibrate(self,readings,configs):
        assert len(readings)==len(configs)
        assert len(readings)>=2
        assert len(readings[0])==len(configs[0])
        #solve the linear regression
        self.coeffs = [0]*len(readings[0])
        self.offsets = [0]*len(readings[0])
        for i in range(len(self.coeffs)):
            A = [[r[i],1] for r in readings]
            b = [c[i] for c in configs]
            AtA = [[sum(r[i]*r[i] for r in readings),sum(r[i] for r in readings)],
                   [sum(r[i] for r in readings),len(readings)]]
            Atb = [sum(r[i]*c[i] for (r,c) in zip(readings,configs)),sum(c[i] for c in configs)]
            print AtA
            print Atb
            AtAinv = inv2x2(AtA)
            #print "AtAinv"
            #print AtAinv
            #rint "AtA*AtAinv"
            #print [vectorops.dot(AtAinv[0],AtA[0]),vectorops.dot(AtAinv[0],AtA[1])]
            #print [vectorops.dot(AtAinv[1],AtA[0]),vectorops.dot(AtAinv[1],AtA[1])]
            self.coeffs[i] = vectorops.dot(AtAinv[0],Atb)
            self.offsets[i] = vectorops.dot(AtAinv[1],Atb)
            print self.coeffs[i],self.offsets[i]
            errori = sum((self.coeffs[i]*r[i]+self.offsets[i]-c[i])**2 for (r,c) in zip(readings,configs))
            print "MSE",i,"in degrees:",errori/len(readings)*math.pi/180
        return
    def load(self,fn=settings.calibrationFile):
        f = open(fn,'r')
        self.coeffs = []
        self.offsets = []
        for line in f.readlines():
            coef,ofs = line.split()
            self.coeffs.append(float(coef))
            self.offsets.append(float(ofs))
        f.close()
    def save(self,fn=settings.calibrationFile):
        f = open(fn,'w')
        for a,b in zip(self.coeffs,self.offsets):
            f.write(str(a)+' '+str(b)+'\n')
        f.close()
        return 

class GLCalibrationGUI(GLRealtimeProgram):
    def __init__(self,world,calibrationConfigs=settings.calibrationConfigs):
        GLRealtimeProgram.__init__(self,"Arduino calibration")
        self.world = world
        self.robot = world.robot(0)
        
        self.readthread = ReadArduinoThread()
        self.readthread.start()
        self.calibrateIndex = -1
        self.calibration = Calibration()
        try:
            self.calibration.load()
        except:
            print "Error loading calibration file"
            self.calibrateIndex = 0
        self.configList = loadCalibrateConfigs(calibrationConfigs)
        self.readingList = []

    def next(self):
        if self.calibrateIndex == len(self.configList):
            return
        self.calibrateIndex += 1
        if self.calibrateIndex == 0:
            return
        self.readingList.append(self.get_reading())
        if self.calibrateIndex == len(self.configList):
            print "Calibrating..."
            self.calibrate()
        return

    def get_reading(self):
        (cnt,timestamp,values)=self.readthread.get_data()
        print values
        return [float(v) for v in values]
    
    def calibrate(self):
        self.calibration = Calibration()
        self.calibration.calibrate(self.readingList,self.configList)
        self.calibration.save()
        print "Saving calibration to arduinoCalibrate.txt"

    def display_screen(self):
        if self.calibrateIndex==len(self.configList):
            return
        glColor3f(0.0,0.0,0.0)
        glDisable(GL_LIGHTING)
        glDisable(GL_TEXTURE_2D)
        glDisable(GL_BLEND)
        glRasterPos2i(20,30)
        if self.calibrateIndex == -1:
            text = 'Press space to begin'
        else:
            text = 'Place your puppet in the given position and press space'
        for c in text:
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,ord(c))
        if self.calibrateIndex >= 0:
            glRasterPos2i(20,50)
            text = ', '.join(str(v) for v in self.get_reading())
            for c in text:
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,ord(c))
        return

    def setConfig(self,q):
        #default: direct control
        #self.robot.setConfig(q)
        #Convert to Klamp't configs
        q = settings.arduinoToKlampt(q)
        self.robot.setConfig(q)
        return

    def display(self):
        #Put your display handler here
        if self.calibrateIndex==len(self.configList) or self.calibrateIndex<0:
            if self.calibration:
                try:
                    q = self.calibration.readingToConfig(self.get_reading())
                    self.setConfig(q)
                except ValueError:
                    pass
        else:
            self.setConfig(self.configList[self.calibrateIndex])
        self.world.drawGL()

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        if c == ' ':
            self.next()
        glutPostRedisplay()

if __name__ == '__main__':
    world = WorldModel()
    if not world.readFile(settings.calibrationWorldFile):
        raise IOError("Unable to read file")
    gui = GLCalibrationGUI(world)
    gui.run()

