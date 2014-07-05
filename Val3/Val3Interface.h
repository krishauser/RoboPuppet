#ifndef VAL3_INTERFACE_H
#define VAL3_INTERFACE_H

#include "SafeTrajClient.h"
#include "Modeling/Robot.h"
#include "Modeling/ParabolicRamp.h"
#include <Timer.h>

/** A class that interfaces between Klamp't and a TrajServer-based device. 
 * The configuration is specified in an XML file given to the Load
 * method, which contains a mapping from Klamp't to/from TrajServer,
 * as well as optional initialization commands.
 *
 * The user connects with the network location of the device and the version
 * of TrajServer that is used.  Currently only TrajServer version 3.0 is
 * supported.
 *
 * The device may be internally emulated, a remote simulator, a real robot,
 * or the Staubli CS8 emulator, depending on the configuration.
 * 
 * The Config/Vector routines handle differences in the mapping between
 * the RobotSim coordinate systems (radians, meters, ccw orientation) and
 * the VAL3 coordinate system (degrees, mm, cw orientation).
 *
 */
class KlamptToVal3Interface
{
 public:
  typedef SafeTrajClient::MotionResult MotionResult;

  KlamptToVal3Interface();
  ~KlamptToVal3Interface();
  bool LoadSetup(const char* setupXml);
  bool Connect(const char* host,int port=1000,double version=1.0);
  bool ConnectInternalEmulator(const char* specsXml);
  void Disconnect();
  void ConfigToTrajClient(const Config& q,TrajClient::Vector& v) const;
  void ConfigFromTrajClient(const TrajClient::Vector& v,Config& q) const;
  void VectorToTrajClient(const Vector& x,TrajClient::Vector& v) const;
  void VectorFromTrajClient(const TrajClient::Vector& v,Vector& x) const;
  void UpdateModelCurrent(Robot* model);

  ///Calling thread must call this periodically
  void SendPoll();

  ///Appends a smooth move to a new configuration from the end of the motion
  ///queue
  MotionResult MoveTo(const Config& q,double rate=1.0,double dt=0.0);

  ///Appends a smooth move starting from the truncated motion queue, starting
  ///at the time dt after the current time.  dt should be some small constant
  ///greater than zero (at least the communication latency + padding)
  MotionResult MoveToImmediate(double dt,const Config& q,double rate=1.0);

  ///Appends a ramp to the end of the motion queue.  Must begin at the end
  ///position/velocity of the motion queue, and must end at zero velocity.
  MotionResult MoveRamp(const ParabolicRamp::ParabolicRampND& ramp);

  ///Truncates the motion queue at tstart and appends a ramp.  Must begin
  ///at the resetted position/velocity of the motion queue, and must end
  ///at zero velocity.
  MotionResult MoveRampImmediate(double tstart,const ParabolicRamp::ParabolicRampND& ramp);

  ///appends multiple ramps to the end of the motion queue.  Must
  ///begin at the end position/velocity of the motion queue, and must
  ///end at zero velocity.
  MotionResult MoveRamps(const std::vector<ParabolicRamp::ParabolicRampND>& ramp);

  ///Truncates the motion queue and appends multiple ramps.  Must begin
  ///at the resetted position/velocity of the motion queue, and must
  ///end at zero velocity.
  MotionResult MoveRampsImmediate(double tstart,const std::vector<ParabolicRamp::ParabolicRampND>& ramp);

  ///basic queries
  bool GetConfig(Config& q);
  bool GetVelocity(Vector& v);
  bool GetTransform(RigidTransform& T);
  bool GetJointLimits(Config& jmin,Config& jmax);
  bool GetVelocityLimits(Vector& vmax);
  bool GetAccelerationLimits(Vector& amax);
  bool GetDecelerationLimits(Vector& dmax);
  bool GetEndConfig(Vector& jend);
  bool GetEndVelocity(Vector& vend);
  double GetCurTime() { return client.EstimateCurrentTime(systemTimer.ElapsedTime()); }
  void AdvanceTime() {
    double t=systemTimer.ElapsedTime();
    client.AdvanceMirrorTime(client.EstimateCurrentTime(t),t);
  }

  ///Sets new (more conservative) joint limits
  void SetJointLimits(const Vector& qmin,const Vector& qmax);
  ///Sets new (more conservative) velocity limits
  void SetSpeedScale(double rate);

  SafeTrajClient client;
  //settings
  string modelName,modelFile;
  //initial setup, used only on initial Connect() step
  string qmin,qmax,vmax,amax,dmax;
  string rate;

  struct IndexMapping
  {
    int index;
    double scale,offset;
  };
  std::vector<IndexMapping> modelFromRobot,robotFromModel;

  //used for managing client time
  Timer systemTimer;
  double nextSendTime;
};


#endif
