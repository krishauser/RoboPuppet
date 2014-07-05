#ifndef VAL3_VIRTUAL_CLIENT_H
#define VAL3_VIRTUAL_CLIENT_H

#include "TrajClient.h"
#include <Timer.h>

/**
 * A virtual implementation of the trajectory client to be used for 
 * offline use, and model-based control.
 */
class VirtualTrajClient 
{
 public:
  typedef TrajClient::Vector Vector;

  VirtualTrajClient();

  bool Connect(const char* host,int port=1000) { return true; }
  void Disconnect();
  bool IsConnected() const { return true; }

  bool Sync(TrajClient& t);
  bool SetupVirtual(const char* xmlFile);

  std::string Echo(const std::string& message) { return message; }
  std::string Version() { return TrajClient::VERSION; }
  double Rate() { return maxRate; }
  bool GetConfig(Vector& q);
  bool GetVelocity(Vector& v);
  bool GetTransform(Vector& t);
  bool GetJointLimits(Vector& jmin,Vector& jmax);
  bool GetVelocityLimits(Vector& vmax);
  bool GetAccelerationLimits(Vector& amax);
  bool GetDecelerationLimits(Vector& dmax);
  int GetMaxSegments();
  int GetCurSegments();
  int GetRemainingSegments();
  double GetCurrentTime();
  double GetTrajEndTime();
  double GetTrajDuration();
  bool GetEndConfig(Vector& jend) const;
  //this is not const because it has to check the current time
  bool GetEndVelocity(Vector& vend);

  bool SetJointLimits(const Vector& jmin,const Vector& jmax);
  bool SetVelocityLimits(const Vector& vmax);
  bool SetAccelerationLimits(const Vector& amax);
  bool SetDecelerationLimits(const Vector& dmax);

  //int AddMilestone(double dt,const Vector& j);
  //bool AddMilestoneQuiet(double dt,const Vector& j);
  //std::vector<int> AppendMilestones(const std::vector<double>& dts,const std::vector<Vector>& q);
  bool AppendMilestonesQuiet(const std::vector<double>& dts,const std::vector<Vector>& q);

  bool ResetTrajectoryAbs(double t);
  bool ResetTrajectoryRel(double dt);
  bool CheckTrajectory();

  ///Nitty gritty stuff
  void AdvanceTime(double t);
  ///Returns an offset from curSegment
  int GetTrajOffset(double t) const;
  ///Returns an absolute segment
  int GetTrajSegment(double t) const;
  double GetTrajTime(int segment) const { return times[segment%(maxSegments+1)]; }
  double& GetTrajTime(int segment) { return times[segment%(maxSegments+1)]; }
  const Vector& GetTrajMilestone(int segment) const { return milestones[segment%(maxSegments+1)]; }
  Vector& GetTrajMilestone(int segment) { return milestones[segment%(maxSegments+1)]; }

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
  std::vector<double> times;
  int curSegment,numSegments;
  Timer virtualTimer;
};

#endif 

