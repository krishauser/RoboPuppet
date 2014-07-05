#ifndef VAL3_CS8_EMULATOR_H
#define VAL3_CS8_EMULATOR_H

#include "TrajClient.h"
#include "Timer.h"

/**
 * A virtual implementation of the CS8 controller for offline use.
 * Useful for model-based planning.
 *
 * The caller is responsible for calling Setup() on an appropriate xml setup
 * file, and periodically calling AdvanceTime() with the system
 * timer (in seconds).
 */
class CS8Emulator
{
 public:
  typedef TrajClient::Vector Vector;

  CS8Emulator();
  bool Setup(const char* xmlFile);
  void AdvanceTime(double t,double tSystem=0.0);
  bool Sync(TrajClient& t);
  void Disconnect();

  std::string Echo(const std::string& message) const { return message; }
  std::string Version() const { return TrajClient::VERSION; }
  double Rate() const { return maxRate; }
  std::string Status() const;
  bool ClearError() { motionError = false; return true; }
  bool GetConfig(Vector& q) const;
  bool GetVelocity(Vector& v) const;
  bool GetTransform(Vector& t) const;
  bool GetJointLimits(Vector& jmin,Vector& jmax) const;
  bool GetVelocityLimits(Vector& vmax) const;
  bool GetAccelerationLimits(Vector& amax) const;
  bool GetDecelerationLimits(Vector& dmax) const;
  int GetMaxSegments() const;
  int GetCurSegments() const;
  int GetRemainingSegments() const;
  double GetCurrentTime() const;
  double GetTrajEndTime() const;
  double GetTrajDuration() const;
  bool GetEndConfig(Vector& jend) const;
  //this is not const because it has to check the current time
  bool GetEndVelocity(Vector& vend) const;

  bool SetJointLimits(const Vector& jmin,const Vector& jmax);
  bool SetVelocityLimits(const Vector& vmax);
  bool SetAccelerationLimits(const Vector& amax);
  bool SetDecelerationLimits(const Vector& dmax);

  int AddMilestone(double t,const Vector& j,const Vector& v);
  //bool AddMilestoneQuiet(double t,const Vector& j,const Vector& v);
  //std::vector<int> AppendMilestones(const std::vector<double>& ts,const std::vector<Vector>& q,const std::vector<Vector>& v);
  bool AppendMilestonesQuiet(const std::vector<double>& ts,const std::vector<Vector>& q,const std::vector<Vector>& v);

  bool ResetTrajectoryAbs(double t);
  bool ResetTrajectoryRel(double dt);
  bool CheckTrajectory();

  ///Produces an estimate of the current mirror time, given the system time
  double EstimateCurrentTime(double tSystem) const { return currentTime+tSystem-currentTimeUpdateTime; }

  ///Nitty gritty stuff
  ///Returns an offset from curSegment
  int GetTrajOffset(double t) const;
  ///Returns an absolute segment
  int GetTrajSegment(double t) const;
  double GetTrajTime(int segment) const { return times[segment%(maxSegments+1)]; }
  double& GetTrajTime(int segment) { return times[segment%(maxSegments+1)]; }
  const Vector& GetTrajMilestone(int segment) const { return milestones[segment%(maxSegments+1)]; }
  Vector& GetTrajMilestone(int segment) { return milestones[segment%(maxSegments+1)]; }
  const Vector& GetTrajTangent(int segment) const { return tangents[segment%(maxSegments+1)]; }
  Vector& GetTrajTangent(int segment) { return tangents[segment%(maxSegments+1)]; }

  void EvalTrajSegment(int i,double t,Vector& x,Vector& v) const;
  void GetTrajConfig(double t,Vector& x) const;
  void GetTrajVelocity(double t,Vector& v) const;

  ///Active joint, velocity, acceleration limits
  Vector qmin,qmax;
  Vector vmax,amax,dmax;
  ///Original joint, velocity acceleration limits
  Vector qmin0,qmax0;
  Vector vmax0,amax0,dmax0;
  ///Controller characteristics
  int maxSegments;
  double maxRate;
  ///Keep a mirror of controller trajectory state
  std::vector<Vector> milestones;
  std::vector<Vector> tangents;
  std::vector<double> times;
  int curSegment,numSegments;
  double currentTime;
  bool motionError;

  //emulated robot servo loop
  Vector qCurrent,dqCurrent,qDesired,dqDesired,qEstimate,dqCommand;
  double lastServoTime;

  //last time the mirror was updated, in the local system time
  double currentTimeUpdateTime;
};

#endif 

