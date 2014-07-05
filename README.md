RoboPuppet: Arduino I/O and Safety Filtering Software

7/5/2014

Authors: Kris Hauser, Anna Eilering


This package contains everything you need to control the Staubli TX90L either in
simulation or through the trajClient3 interface on the physical robots.


DEPENDENCIES:
- Python 2.7
- Klampt (http://klampt.org) Python API 
- CMake


BUILDING
- This assumes that the RoboPuppet directory is in the same parent directory as Klampt. 
  If not, edit the KLAMPT_ROOT variable in CMakeLists.txt to point to the Klamp't directory.
- Run 'cmake .'
- Run 'make' to build the SafeSerialClient and Val3SerialClient programs


CONCEPTS:

The key concepts of the method are shown in the diagram below.  Each element in square
brackets [ ] is a processor, and each element in curved brackets ( ) is a datatype.

  ARDUINO READING AND FILTERING:
    [ Arduino reader ] -> (Raw signals) -> [ Filter ] -> (Filtered signals)

  LEARNING CALIBRATION:
                     (Calibration configurations)
                                 |
                                 v
    (Filtered signals) -> [Visual calibrator] -> (Calibration map)

  APPLYING CALIBRATION:
    (Filtered signals) -> [Calibration] -> (Joint angle objective)

  PYTHON-KLAMPT RELAY:
    (Joint angle objective) -> [Serial sender] -> (JSON objective message) ->
    [Serial receiver] -> (Joint angle objective)

  DYNAMICS AND COLLISION FILTERING
    (Joint angle objective) -> [Motion planner] -> (Motion commands) -> [Robot]


TO LEARN A CALIBRATION:
- Connect the Arduino, and edit the defaultPort member in settings.py to reflect
  the port to which it is attached to.
- Run "python calibrate.py".  This will save a file arduinoCalibrate.txt which will
  be used by the other programs.
- To test your calibration in simulation, run "python arduinoSimControl.py"


TO RUN SYNTHETIC INPUT ON A SIMULATED ROBOT
- Run "python syntheticSerialRelay.py"
- Run "./SafeSerialClient data/tx90room.xml"


TO RUN REAL INPUT ON A SIMULATED ROBOT
- Connect the Arduino, and note which port it is attached to.
- Run "python arduinoSerialRelay.py -port=/dev/tty[PORT NUMBER]"
- Run "./SafeSerialClient data/tx90room.xml"


TO RUN REAL INPUT ON THE REAL ROBOT
- Connect the Arduino, and note which port it is attached to.
- Run "python arduinoSerialRelay.py -port=/dev/tty[PORT NUMBER]"
- Run "./Val3SerialClient data/tx90room.xml Val3/tx90l_specs.xml Val3/tx90right_medspeed.xml"


TO ADAPT THIS TO A NEW ROBOT
- You will need to define your own calibration configurations, as well as the map from
  RoboPuppet inputs to Klamp't configurations.  You will need to edit settings.py to do so.
  - Calibration configurations are in tx90/calibrate.configs.  Saving configurations can
    be done in RobotPose for your new robot.
  - The re-indexing from RoboPuppet inputs to Klamp't configurations is done in the
    arduinoToKlampt() function in settings.py.  You'll need to edit this function.
- You will also need to set up your own world file.  See the Klamp't documentation for
  more details.