#ifndef SAFE_TRAJ_CLIENT_H
#define SAFE_TRAJ_CLIENT_H

#include "TrajClient.h"
#include "CS8Emulator.h"
#include <Timer.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>

/**
 * A TrajClient API that avoids violating joint, velocity, and
 * acceleration constraints.  Trajectories are required to terminate at a
 * stationary configuration.
 *
 * Note: safety is only guaranteed between method calls.  If the connection to
 * the server is lost during a motion command, then bad things may happen!
 * Also, the user is responsible for avoiding environment collisions and
 * self-collisions.
 *
 * Nonblocking mode is a limited form of nonblocking when large motion
 * commands are sent to the controller.  If the client is blocking, then
 * a large motion command is broken into pieces, and the function call
 * will wait until all pieces are executed.  Otherwise, the command simply
 * places a request to the send queue, and the user of this class is
 * responsible for calling SendPoll() repeatedly, or FlushSendQueue() once,
 * to ensure that all motion commands are sent.
 */
class SafeTrajClient
{
 public:
  typedef TrajClient::Vector Vector;

  SafeTrajClient(const char* host=NULL,int port=1000);
  bool LogBegin(const char* fn="trajclient.log");
  bool LogEnd();
  bool Connect(const char* host,int port=1000);
  void Disconnect();
  bool IsConnected() const;

  /// A virtual TrajClient can be used for debugging purposes.  No
  /// commands are actually sent to the network
  bool SetVirtual(const char* xmlFile);

  ///Calibrates the client-server latency using an average round-trip time
  void CalibrateLatency(int iters=10);
  double GetAverageLatency() const { return latency; }

  ///Sets nonblocking mode (send queue must be clear)
  void SetNonblocking(bool nonblock);

  /**
   * Returns a safe version of the TrajClient object used by this
   * object.  Motion commands and setting joint/vel/acc limits are not
   * allowed from this object.
   */
  ReadOnlyTrajClient Client();

  /// Returns the mirror
  const CS8Emulator& Mirror() { return mirror; }

  ///These convenience functions duplicate the functionality of TrajClient,
  ///but may also reduce network traffic.  They also allow the use of a virtual
  ///controller.
  std::string Echo(const std::string& message) { return (virtualController?mirror.Echo(message):t.Echo(message)); }
  std::string Version() { return (virtualController?mirror.Version():t.Version()); }
  double Rate() { return mirror.maxRate; }
  bool GetConfig(Vector& q) { return (virtualController?mirror.GetConfig(q):t.GetConfig(q)); }
  bool GetVelocity(Vector& v) { return (virtualController?mirror.GetVelocity(v):t.GetVelocity(v)); }
  bool GetTransform(Vector& x) { return (virtualController?mirror.GetTransform(x):t.GetTransform(x)); }
  bool GetJointLimits(Vector& jmin,Vector& jmax) { return mirror.GetJointLimits(jmin,jmax); }
  bool GetVelocityLimits(Vector& vmax) { return mirror.GetVelocityLimits(vmax); }
  bool GetAccelerationLimits(Vector& amax) { return mirror.GetAccelerationLimits(amax); }
  bool GetDecelerationLimits(Vector& dmax) { return mirror.GetDecelerationLimits(dmax); }
  int GetMaxSegments() { return mirror.GetMaxSegments(); }
  int GetCurSegments() { return (virtualController?mirror.GetCurSegments():t.GetCurSegments()); }
  int GetRemainingSegments() { return (virtualController?mirror.GetRemainingSegments():t.GetRemainingSegments()); }
  double GetCurrentTime() { return (virtualController?mirror.GetCurrentTime():t.GetCurrentTime()); }
  double GetTrajEndTime() { return (virtualController?mirror.GetTrajEndTime():t.GetTrajEndTime()); }
  double GetTrajDuration() { return (virtualController?mirror.GetTrajDuration():t.GetTrajDuration()); }
  bool GetEndConfig(Vector& jend);
  bool GetEndVelocity(Vector& vend);
  bool CheckTrajectory() { return (virtualController?mirror.CheckTrajectory():t.CheckTrajectory()); }

  ///Sets new (more conservative) joint limits
  void SetJointLimits(const Vector& qmin,const Vector& qmax);

  ///Sets new (more conservative) velocity/accel/decel limits
  void SetVelocityLimits(const Vector& vmax);
  void SetAccelerationLimits(const Vector& amax);
  void SetDecelerationLimits(const Vector& dmax);

  /**
   * Convenience function: Sets a maximum velocity and acceleration multiplier.
   * rate must be in the range (0,1]
   */
  void SetSpeedScale(double rate);

  ///Returns the commanded configuration at absolute time t (t >= current time)
  void GetTrajConfig(double t,Vector& x) const;

  ///Returns the commanded velocity at absolute time t (t >= current time)
  void GetTrajVelocity(double t,Vector& x) const;

  ///Adds a braking trajectory starting at relative time dt
  bool Brake(double tSystem,double dt);

  ///Adds a braking trajectory starting at absolute time t
  bool BrakeAbs(double t);

  /**
   * Abruptly resets the trajectory at time t.
   * Note: unsafe, unless velocity=0 at time t, or if this is immediately
   * followed by addMilestone commands that slow the robot gradually.
   */
  bool ResetTrajectoryAbs(double t);

  /**
   * Abruptly resets the trajectory at time dt after the command is recieved.
   * Note: unsafe, unless velocity=0 at time dt, or if this is immediately
   * followed by addMilestone commands that slow the robot gradually.
   */
  bool ResetTrajectoryRel(double dt);

  /**
   * Appends a smooth linear move from the trajectory queue's end
   * to the configuration q.
   *
   * If dt is provided, then the move is attempted so that it ends
   * exactly at dt.  If dt is too short to achieve the motion feasibly,
   * then a ValueError is raised.
   * 
   * If rate is provided, then the robot's
   * maximum speed and acceleration is multiplied by rate.
   */
  bool MoveTo(const Vector& q,double rate=1.0,double dt=0.0);

  /**
   * Interrupts the current trajectory at time dt and appends a
   * a smooth move from the cutoff point to the configuration q.
   *
   * If rate is provided, then the robot's
   * maximum speed and acceleration is multiplied by rate.
   */
  bool MoveToImmediate(double tSystem,double dt,const Vector& q,double rate=1.0);

  /**
   * Defines a complete move through the configuration/velocity
   * milestones q,dq.  If the segment time durations dt are provided,
   * then it tries to meet the milestone at the given times.
   */
  bool MoveTraj(const std::vector<Vector>& q,const std::vector<Vector>& dq,const std::vector<double>& dt,double rate=1.0);
  bool MoveTraj(const std::vector<Vector>& q,const std::vector<Vector>& dq,double rate=1.0);
  bool MoveTraj(const std::vector<Vector>& q,const std::vector<double>& dt,double rate=1.0);
  bool MoveTraj(const std::vector<Vector>& q,double rate=1.0);

  /**
   * Sends the trajectory f(t), for t in [0,tmax], to the
   * controller at the maximum rate.  If f(t) is not safe, then false is
   * returned and the trajectory is left unsent.
   */
  template <class TrajectoryFunction>
  bool AddTrajectory(TrajectoryFunction f,double tmax);

  /**
   * Resets the path at tstart and sends the trajectory f(t), for t
   * in [0,tmax], to the controller at the maximum rate.  If f(t) is not
   * safe to execute starting at tstart, then false is returned and the
   * trajectory is left unsent.
   */
  template <class TrajectoryFunction>
  bool AddTrajectoryImmediate(double tstart,TrajectoryFunction f,double tmax);

  bool CheckState(const Vector& q) const;
  bool CheckState(const Vector& q,const Vector& dq) const;
  bool CheckState(const Vector& q,const Vector& dq,const Vector& ddq) const;

  /// Set tstart <= 0
  template <class TrajectoryFunction>
  bool CheckTrajectory(double tstart,TrajectoryFunction f,double tmax);

  /// Estimates the current server time from the local system clock
  double EstimateCurrentTime(double tSystem) { return mirror.EstimateCurrentTime(tSystem); }

  /**
   * In nonblocking send mode, this function should be called repeatedly
   * in a loop.  It will return the approximate amount of time before the
   * motion queue is emptied.
   *
   * Note: non-thread safe.  Use a lock around all motion commands and
   * SendPoll if you wish to use multithreaded sending.
   */
  double SendPoll(double tSystem);

  /** 
   * This needs to be called once in a while.  Returns the server time.
   */
  double SyncTimers(double tSystem);
  void AdvanceMirrorTime(double tServer,double tSystem) { mirror.AdvanceTime(tServer,tSystem); }

  /**
   * In nonblocking send mode, this function will block until all motion
   * commands are sent.
   */
  void FlushSendQueue();

 private:
  /**
   * Sends the trajectory f(t), for t in [0,tmax], to the
   * controller at intervals dt.  (If dt is not specified, it sends
   * milestones at the maximum rate.)  The trajectory is appended to the
   * current motion queue.
   *
   * *Important note*
   * *The caller is responsible for ensuring that the trajectory is safe!*
   *
   * If the controller's buffer will not allow tmax/dt
   * milestones to be sent at once, this routine will block and send them
   * off in batches until complete.
   * 
   * If tstart is provided and is nonzero, the current motion queue is clipped
   * at tstart.
   */
  template <class TrajectoryFunction>
  bool SendTrajectory(double tstart,TrajectoryFunction f,double tmax,double dt=0.0);

  //process the send queue given the current state of the mirror
  //returns true if it's done, false otherwise
  bool DoSendMilestones();

  TrajClient t;
  CS8Emulator mirror;
  ///Controller characteristics
  double latency;
  int milestoneBatch;
  bool nonblock;
  bool virtualController;

  ///Temporary buffer for nonblocking mode
  std::vector<Vector> sendqs;
  std::vector<double> sendts;
  int sendIndex;
};

template <class TrajectoryFunction>
bool SafeTrajClient::AddTrajectory(TrajectoryFunction f,double tmax)
{
  if(!this->CheckTrajectory(0,f,tmax)) return false;
  return this->SendTrajectory(0,f,tmax);
}

template <class TrajectoryFunction>
bool SafeTrajClient::AddTrajectoryImmediate(double tstart,TrajectoryFunction f,double tmax)
{
  if(!this->CheckTrajectory(tstart,f,tmax)) return false;
  return this->SendTrajectory(tstart,f,tmax);
}

template <class TrajectoryFunction>
bool SafeTrajClient::CheckTrajectory(double tstart,TrajectoryFunction f,double tmax) 
{
  assert(tmax >= 0);
  double dt = 1.0/mirror.maxRate;
  Vector qprev;
  Vector dqprev;
  if(tstart <= 0) {  //add to end of motion queue
    GetEndConfig(qprev);
    GetEndVelocity(dqprev);
  }
  else {
    if(tstart < GetCurrentTime()) {
      fprintf(stderr,"CheckPath: reset is before current time\n");
      return false;
    }
    if(tstart > mirror.GetTrajEndTime() && sendIndex < (int)sendts.size()) {
      fprintf(stderr,"CheckPath: reset is after current segments, and I'm still sending\n");
      abort();
      return false;
    }
    mirror.GetTrajConfig(tstart,qprev);
    mirror.GetTrajVelocity(tstart,dqprev);
  }
  Vector q(qprev.size()), dq(qprev.size()), ddq(qprev.size());
  //this is an unnecessary amount of checking
  /*
  q=f(0);
  dq=f.Derivative(0);
  for(size_t i=0;i<qprev.size();i++) {
    if(fabs(qprev[i]-q[i]) > 1e-3 || fabs(dqprev[i]-dq[i]) > 1e-2) {
      fprintf(stderr,"CheckTrajectory: Trajectory doesn't start at current end config\n");
      fprintf(stderr,"\tq\tdq\tf(0)\tf'(0)\n");
      for(size_t i=0;i<qprev.size();i++) {
	fprintf(stderr,"\t%.03g\t%.03g\t%.03g\t%.03g\n",qprev[i],dqprev[i],q[i],dq[i]);
      }
      return false;
    }
  }
  */
  double t = dt;
  while (t < tmax) {
    q = f(t);
    assert(q.size() == qprev.size());
    for(size_t i=0;i<q.size();i++) {
      dq[i] = (q[i]-qprev[i])/dt;
      ddq[i] = (dq[i]-dqprev[i])/dt;
    }
    if(!this->CheckState(q,dq,ddq)) {
      fprintf(stderr,"CheckPath: State infeasible at time %g/%g\n",t,tmax);
      fprintf(stderr,"\tq\tdq\tddq\t\tqprev\tdqprev\n");
      for(size_t i=0;i<q.size();i++) {
	fprintf(stderr,"\t%.03g\t%.03g\t%.03g\t\t%.03g\t%.03g\n",q[i],dq[i],ddq[i],qprev[i],dqprev[i]);
      }
      return false;
    }
    t += dt;
    qprev = q;
    dqprev = dq;
  }

  //check last point/velocity
  q = f(tmax);
  assert(q.size() == qprev.size());
  for(size_t i=0;i<q.size();i++) {
    dq[i] = (q[i]-qprev[i])/dt;
    ddq[i] = (dq[i]-dqprev[i])/dt;
  }
  if(!this->CheckState(q,dq,ddq)) {
    fprintf(stderr,"CheckPath: End state infeasible at time %g\n",tmax);
      fprintf(stderr,"\tq\tdq\tddq\t\tqprev\tdqprev\n");
    for(size_t i=0;i<q.size();i++) {
	fprintf(stderr,"\t%.03g\t%.03g\t%.03g\t\t%.03g\t%.03g\n",q[i],dq[i],ddq[i],qprev[i],dqprev[i]);
    }
    return false;
  }
        
  //check last acceleration
  for(size_t i=0;i<q.size();i++) {
    ddq[i] = dq[i]/dt;
    dq[i] = 0;
  }
  if(!this->CheckState(q,dq,ddq)) {
    fprintf(stderr,"CheckPath: Post-end state infeasible at time %g\n",tmax+dt);
      fprintf(stderr,"\tq\tdq\tddq\t\tqprev\tdqprev\n");
    for(size_t i=0;i<q.size();i++) {
	fprintf(stderr,"\t%.03g\t%.03g\t%.03g\t\t%.03g\t%.03g\n",q[i],dq[i],ddq[i],qprev[i],dqprev[i]);
    }
    return false;
  }
  return true;
}

template <class TrajectoryFunction>
bool SafeTrajClient::SendTrajectory(double tstart,TrajectoryFunction f,double tmax,double dt)
{
  //250Hz
  //dt = std::max(dt,1.0/mirror.maxRate);
  //50 Hz
  dt = std::max(dt,5.0/mirror.maxRate);
  // printf("Sending %g seconds of trajectory at rate %g\n",tmax,dt);
  std::vector<double> ts((int)ceil(tmax/dt),dt);
  std::vector<Vector> qs(ts.size());
  if(!ts.empty()) {
    ts.back() = tmax - dt*(ts.size()-1);
    double t = 0;
    for(size_t i=0;i<ts.size();i++) {
      assert(ts[i] > 0);
      t += ts[i];
      qs[i] = f(t);
    }
    assert(fabs(t - tmax) < 1e-6);
    //add another extra point in time?
    ts.push_back(ts.back()+dt);
    qs.push_back(qs.back());
    //printf("Appending %d milestones, %g duration\n",qs.size(),tmax);
  }

  //Timer timer;
  if(tstart != 0) {
    //reset the trajectory starting from tstart
    if(!ResetTrajectoryAbs(tstart)) {
      fprintf(stderr,"ResetTrajectoryAbs(%g) failed\n",tstart);
      return false;
    }
    //printf("Time for resetTrajectory: %g\n",timer.ElapsedTime());
  }

  sendqs.erase(sendqs.begin(),sendqs.begin()+sendIndex);
  sendts.erase(sendts.begin(),sendts.begin()+sendIndex);
  sendIndex = 0;
  sendqs.insert(sendqs.end(),qs.begin(),qs.end());
  sendts.insert(sendts.end(),ts.begin(),ts.end());

  if(tstart != 0) {
    //send an immediate batch of milestones
    //SendPoll();
    //bool res=DoSendMilestones();
    //printf("Send immediate batch result %d\n",(int)res);
    //printf("Time for reset + sendMilestones: %g\n",timer.ElapsedTime());
    //t.GetCurrentTime();
    //printf("Time for becoming responsive: %g\n",timer.ElapsedTime());
  }

  //if not nonblocking, wait until the trajectory is sent
  if(!nonblock) {
    FlushSendQueue();
  } 
  return true;
}


#endif
