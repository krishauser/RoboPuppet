#ifndef VAL3_ROBOT_INTERFACE_H
#define VAL3_ROBOT_INTERFACE_H

#include "Interface/RobotInterface.h"
#include "Val3Interface.h"

/** @brief Uses TrajClient3 serial interface to communicate with Staubli
 * robots.
 *
 * Immediate commands are delayed by some delay depending on communication
 * latency.  The maximum of minimumLatency and
 * resetLatencyMultiplier*averageLatency is taken as the delay value.
 */
class Val3MotionQueueInterface : public MotionQueueInterface
{
 public:
  Val3MotionQueueInterface(KlamptToVal3Interface& _val3)
    :val3(_val3),minimumLatency(0.08),resetLatencyMultiplier(1.5)
    {}
  virtual bool HadExternalChange() {
    return val3.client.CheckAndResetErrors();
  }
  bool UpdateRobotLimits(Robot& robot) {
    //load model properties from val3 server, update robot model
    TrajClient::Vector vjmin,vjmax,vvmax,vamax;
    if(!val3.client.GetJointLimits(vjmin,vjmax)) {
      fprintf(stderr,"Unable to retrieve joint limits from Val3 server\n");
      return false;
    }
    if(!val3.client.GetVelocityLimits(vvmax)) {
      fprintf(stderr,"Unable to retrieve velocity limits from Val3 server\n");
      return false;
    }
    if(!val3.client.GetAccelerationLimits(vamax)) {
      fprintf(stderr,"Unable to retrieve velocity limits from Val3 server\n");
      return false;
    }
    Vector jmin,jmax,vmax,amax;
    val3.ConfigFromTrajClient(vjmin,jmin);
    val3.ConfigFromTrajClient(vjmax,jmax);
    val3.VectorFromTrajClient(vvmax,vmax);
    val3.VectorFromTrajClient(vamax,amax);
    for(int i=0;i<jmin.n;i++) {
      if(jmin[i] > jmax[i])
	Swap(jmin[i],jmax[i]);
      Real d=(jmax[i]-jmin[i]);
      jmin[i] += d*0.001;
      jmax[i] -= d*0.001;
    }
    for(int i=0;i<vmax.n;i++)
      vmax[i] = Abs(vmax[i])*0.99;
    for(int i=0;i<amax.n;i++)
      amax[i] = Abs(amax[i])*0.99;
    robot.qMin = jmin;
    robot.qMax = jmax;
    robot.velMin.setNegative(vmax);
    robot.velMax = vmax;
    robot.accMax = amax;
    return true;
  }
  virtual Real GetCurTime() { return val3.GetCurTime(); }
  virtual void GetCurConfig(Config& x) { val3.GetConfig(x); }
  virtual void GetCurVelocity(Config& dx) { val3.GetVelocity(dx); }
  virtual Real GetEndTime() { return val3.client.GetEndTime(); }
  virtual void GetEndConfig(Config& x) { val3.GetEndConfig(x); }
  virtual void GetEndVelocity(Config& dx) { val3.GetEndVelocity(dx); }
  virtual void GetConfig(Real t,Config& x) { 
    TrajClient::Vector vx;
    val3.client.Mirror().GetTrajConfig(t,vx);
    val3.ConfigFromTrajClient(vx,x);
  }
  virtual MotionResult SendMilestone(const Config& x) {
    SafeTrajClient::MotionResult res=val3.MoveTo(x);
    return ToMotionResult(res);
  }
  virtual MotionResult SendMilestoneImmediate(const Config& x) {
    double delay = Max(minimumLatency,val3.client.GetAverageLatency()*resetLatencyMultiplier);
    SafeTrajClient::MotionResult res = val3.MoveToImmediate(delay,x);
    return ToMotionResult(res);
  }
  virtual MotionResult SendPathImmediate(Real tbreak,const ParabolicRamp::DynamicPath& path) {
    double delay = Max(minimumLatency,val3.client.GetAverageLatency()*resetLatencyMultiplier);
    if(tbreak < GetCurTime()+delay) {
      printf("SendPathImmediate: Warning, break time %g < time %g + estimated delay %g\n",tbreak,GetCurTime(),delay);
      //TEMP: ignore predicted failures
      //return TransmitError;
    }

    if(path.Empty()) {
      SafeTrajClient::MotionResult res=val3.client.ResetTrajectoryAbs(tbreak);
      return ToMotionResult(res);
    }
    else {
      assert(path.IsValid());
      SafeTrajClient::MotionResult res=val3.MoveRampsImmediate(tbreak,path.ramps);
      if(res != SafeTrajClient::Success) fprintf(stderr,"MoveRampsImmediate failed, code %d\n",res);
      return ToMotionResult(res);
    }
  }
  static MotionResult ToMotionResult(SafeTrajClient::MotionResult res)
  {
    if(res==SafeTrajClient::Success) return Success;
    if(res==SafeTrajClient::InvalidParams) return InvalidParams;
    if(res==SafeTrajClient::FailedCheck) return FailedCheck;
    if(res==SafeTrajClient::TransmitError) return TransmitError;
    return InvalidParams;
  }


  KlamptToVal3Interface& val3;
  double minimumLatency,resetLatencyMultiplier;
};

#endif
