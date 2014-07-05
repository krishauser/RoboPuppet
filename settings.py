def arduinoToKlampt(q):
    """Maps an Arduino configuration to Klamp't"""
    return [0]+q+[0]*(7-1-len(q))

def klamptToArduino(q):
    """Maps a Klamp't configuration to Arduino"""
    return q[1:6]

defaultPort = '/dev/ttyS7'

calibrationWorldFile = '../Klampt/data/tx90scenario0.xml'

calibrationConfigs = 'tx90/calibration.configs'

calibrationFile = 'tx90/arduinoCalibrate.txt'

simControlWorldFile = 'data/tx90room.xml'
