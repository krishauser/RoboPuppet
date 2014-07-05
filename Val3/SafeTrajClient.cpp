#include "SafeTrajClient.h"
#include <utils/threadutils.h>
#include <spline/PiecewisePolynomial.h>
#include <math/misc.h>
#include "Modeling/ParabolicRamp.h"
#include "Modeling/Conversions.h"
#include "Timer.h"
#include <assert.h>
using namespace std;
using namespace Spline;

double kVelSafetyMultiplier = 0.99;
double kAccSafetyMultiplier = 0.99;

char* AscLocalTime();

void HInterpolate(double t1,double q1,double v1,
		  double t2,double q2,double v2,
		  double t,double& qt,double& dqt);

//defined in CS8Emulator.cpp
void HInterpolate(double t1,const CS8Emulator::Vector& q1,const CS8Emulator::Vector& v1,
		  double t2,const CS8Emulator::Vector& q2,const CS8Emulator::Vector& v2,
		  double t,CS8Emulator::Vector& qt,CS8Emulator::Vector& dqt);

bool CheckHermite(double t1,const CS8Emulator::Vector& q1,const CS8Emulator::Vector& v1,
		  double t2,const CS8Emulator::Vector& q2,const CS8Emulator::Vector& v2,
		  CS8Emulator::Vector& qmin,CS8Emulator::Vector& qmax,
		  CS8Emulator::Vector& vmax,CS8Emulator::Vector& amax)
{
  assert(t2 >= t1);
  double dt=t2-t1;
  double cq1a = -6.0/dt;
  double cq1b = 6.0/dt;
  double cv1a = -4.0;
  double cv2a = -2.0;
  double cv1b = 2.0;
  double cv2b = 4.0;


  for(size_t i=0;i<q1.size();i++) {
    if(q1[i] < qmin[i] || q1[i] > qmax[i]) {
      fprintf(stderr,"Start position %d at time %g out of bounds %g in [%g,%g]\n",i,t1,q1[i],qmin[i],qmax[i]);
      return false;
    }
    if(q2[i] < qmin[i] || q2[i] > qmax[i]) {
      fprintf(stderr,"End position %d at time %g out of bounds %g in [%g,%g]\n",i,t2,q2[i],qmin[i],qmax[i]);
      return false;
    }

    if(fabs(v1[i]) > vmax[i]) {
      fprintf(stderr,"Start velocity %d at time %g exceeds bound %g > %g\n",i,t1,v1[i],vmax[i]);
      return false;
    }
    if(fabs(v2[i]) > vmax[i]) {
      fprintf(stderr,"End velocity %d at time %g exceeds bound %g > %g\n",i,t2,v2[i],vmax[i]);
      return false;
    }
 
    if(t2 == t1) {
      if(fabs(q1[i]-q2[i])>1e-3) {
	fprintf(stderr,"Null spline is discontinuous in position %d: %g vs %g\n",i,q1[i],q2[i]);
	return false;
      }
      if(fabs(v1[i]-v2[i])>1e-3) {
	fprintf(stderr,"Null spline is discontinuous in velocity %d: %g vs %g\n",i,v1[i],v2[i]);
	return false;
      }
      continue;
    }

    double a1 = cq1a*(q1[i]-q2[i])+cv1a*v1[i]+cv2a*v2[i];
    double a2 = cq1b*(q1[i]-q2[i])+cv1b*v1[i]+cv2b*v2[i];
    if(fabs(a1) > amax[i]*dt) {
      fprintf(stderr,"Start accel %d at time %g exceeds bound %g > %g\n",i,t1,a1/dt,amax[i]);
      printf("coeffs %g %g %g\n",cq1a/dt,cv1a/dt,cv2a/dt);
      printf("Values %g %g %g %g\n",q1[i],q2[i],v1[i],v2[i]);
      return false;
    }
    if(fabs(a2) > amax[i]*dt) {
      fprintf(stderr,"End accel %d at time %g exceeds bound %g > %g\n",i,t2,a2/dt,amax[i]);
      printf("coeffs %g %g %g\n",cq1b/dt,cv1b/dt,cv2b/dt);
      printf("Values %g %g %g %g\n",q1[i],q2[i],v1[i],v2[i]);
      return false;
    }
    //check for extrema of position
    //double dcq1 = (6.0*u2-6.0*u)/dt; = 6u(u-1)
    //double dcq2 = -(6.0*u2-6.0*u)/dt; = 6u(1-u)
    //double dcv1 = 3.0*u2-4.0*u+1.0; = 3 u(u-1) + (1-u)
    //double dcv2 = 3.0*u2-2.0*u;     = 3 u(u-1) + u
    //find all u such that
    //dcq1(q2-q1) = dcv1*v1 + dcv2*v2
    //u(u-1)3(v1+v2 - 2(q2-q1)) + (1-u) v1 + u v2 = 0
    double a=3.0*(v1[i]+v2[i] + 2.0*(q1[i]-q2[i]));
    double b = v2[i]-v1[i]-a;
    double c = v2[i];
    double u1,u2;
    int res=Math::quadratic(a,b,c,u1,u2);
    if (res >= 1 && u1 >= 0 && u1 <= 1) {
      double x,v;
      HInterpolate(t1,q1[i],v1[i],t2,q2[i],v2[i],t1+u1*(t2-t1),x,v);
      if(x < qmin[i] || x > qmax[i]) {
	fprintf(stderr,"Intermediate position %d at time %g out of bounds %g in [%g,%g]\n",i,t1+u1*(t2-t1),x,qmin[i],qmax[i]);
	return false;
      }
    }
    if (res >= 2 && u2 >= 0 && u2 <= 1) {
      double x,v;
      HInterpolate(t1,q1[i],v1[i],t2,q2[i],v2[i],t1+u2*(t2-t1),x,v);
      if(x < qmin[i] || x > qmax[i]) {
	fprintf(stderr,"Intermediate position %d at time %g out of bounds %g in [%g,%g]\n",i,t2+u1*(t2-t1),x,qmin[i],qmax[i]);
	return false;
      }
    }

    //check for other extrema of velocity function
    if(a1*a2 < 0) {
      double u = a1/(a1-a2);
      double x,v;
      HInterpolate(t1,q1[i],v1[i],t2,q2[i],v2[i],t1+u*(t2-t1),x,v);
      if(fabs(v) > vmax[i]) {
	fprintf(stderr,"Intermediate accel %d at time %g exceeds bound %g > %g\n",i,t1+u*(t2-t1),v,vmax[i]);
	return false;
      }
    }
  }
  return true;
}

void DiscretizeTrajectory(const Spline::PiecewisePolynomialND& f,vector<double>& t,vector<vector<double> >& q,vector<vector<double> >& v)
{
  vector<double> times(1,f.StartTime());
  double epsilon = 1e-6;
  for(size_t i=0;i<f.elements.size();i++) {
    size_t k=0;
    for(size_t j=0;j<f.elements[i].times.size();j++) {
      //advance k s.t. times[k] <= f.elements[i].times[j] < times[k+1]
      while(k+1<times.size() && times[k+1]+epsilon < f.elements[i].times[j]) {
	k++;
      }
      if(f.elements[i].times[j] > times[k]+epsilon && (k+1 == times.size() || f.elements[i].times[j] < times[k+1]-epsilon)) {
	//printf("Inserting %g after %g and before %g\n",f.elements[i].times[j],times[k],(k+1 == times.size()? -1 : times[k+1]));
	times.insert(times.begin()+k+1,f.elements[i].times[j]);
	k++;
      }
    }
  }
  for(size_t i=0;i<f.elements.size();i++) {
    for(size_t j=0;j<f.elements[i].times.size();j++) {
      bool found=false;
      for(size_t k=0;k<times.size();k++)
	if(fabs(times[k]-f.elements[i].times[j]) <= epsilon) {
	  found=true;
	  break;
	}
      if(!found) {
	fprintf(stderr,"DiscretizeTrajectory: error, failed to get time %g in element %d\n",f.elements[i].times[j],i);
	fprintf(stderr,"Found times: ");
	for(size_t k=0;k<times.size();k++)
	  fprintf(stderr,"%g ",times[k]);
	fprintf(stderr,"\n");
	fprintf(stderr,"element times: ");
	for(size_t j=0;j<f.elements[i].times.size();j++) 
	  fprintf(stderr,"%g ",f.elements[i].times[j]);
	fprintf(stderr,"\n");	  
	abort();
      }
    }
  }
  t.resize(times.size());
  q.resize(times.size());
  v.resize(times.size());
  t = times;
  for(size_t i=0;i<times.size();i++) {
    q[i] = f.Evaluate(times[i]);
    v[i] = f.Derivative(times[i]);
  }
}


SafeTrajClient::SafeTrajClient(const char* host,int port)
  :milestoneBatch(100),nonblock(false),virtualController(false),sendReset(-1),sendIndex(0)
{
  if(host) {
    bool res=Connect(host,port);
    if(!res) fprintf(stderr,"SafeTrajClient failed to connect to %s:%d\n",host,port);
  }
}

bool SafeTrajClient::LogBegin(const char* fn)
{
  return t.LogBegin(fn);
}

bool SafeTrajClient::LogEnd()
{
  return t.LogEnd();
}

void SafeTrajClient::Disconnect()
{
  virtualController = false;
  t.Disconnect();
  mirror.Disconnect();

  sendReset = -1;
  sendIndex = 0;
  sendqs.clear();
  sendvs.clear();
  sendts.clear();
}

bool SafeTrajClient::IsConnected() const
{
  return t.IsConnected();
}

bool SafeTrajClient::Connect(const char* host,int port)
{
  assert(!t.IsConnected());
  assert(!virtualController);

  if(!t.Connect(host,port)) return false;
  if(!mirror.Sync(t)) return false;
  latency = 0;
  return true;
}

bool SafeTrajClient::SetVirtual(const char* xmlFile)
{
  assert(!IsConnected());
  if(!mirror.Setup(xmlFile)) return false;
  virtualController = true;
  latency = 0;
  return true;
}

ReadOnlyTrajClient SafeTrajClient::Client()
{
  return ReadOnlyTrajClient(this->t);
}

void SafeTrajClient::CalibrateLatency(int iters)
{
  if(virtualController) return;
  printf("SafeTrajClient: Calibrating latency... \n");
  Timer timer;
  for (int i=0;i<iters;i++)
    this->t.Echo("");
  this->latency = timer.ElapsedTime()/iters;
  printf("   result %g\n",this->latency);
}

void SafeTrajClient::SetNonblocking(bool nonblock)
{
  FlushSendQueue();
  this->nonblock = nonblock;
}

bool SafeTrajClient::CheckAndResetErrors()
{
  if(this->t.Status()=="motionError") {
    sendqs.clear();
    sendvs.clear();
    sendts.clear();
    sendIndex = 0;
    sendReset = -1;
    printf("SafeTrajClient: Motion error detected, syncing CS8 mirror...\n");
    while(!mirror.Sync(t)) {
      printf("SafeTrajClient: Error syncing CS8 mirror, retrying in 2s...\n");
      ThreadSleep(2.0);
    }
    return true;
  }
  return false;
}

void SafeTrajClient::SetJointLimits(const Vector& qmin,const Vector& qmax)
{
  mirror.SetJointLimits(qmin,qmax);
  if(!virtualController) 
    this->t.SetJointLimits(mirror.qmin,mirror.qmax);
}


void SafeTrajClient::SetVelocityLimits(const Vector& vmax)
{
  mirror.SetVelocityLimits(vmax);
  if(!virtualController) 
    this->t.SetVelocityLimits(mirror.vmax);
}

void SafeTrajClient::SetAccelerationLimits(const Vector& amax)
{
  mirror.SetAccelerationLimits(amax);
  if(!virtualController) 
    this->t.SetAccelerationLimits(mirror.amax);
}

void SafeTrajClient::SetDecelerationLimits(const Vector& dmax)
{
  mirror.SetDecelerationLimits(dmax);
  if(!virtualController) 
    this->t.SetDecelerationLimits(mirror.dmax);
}

void SafeTrajClient::SetSpeedScale(double rate)
{
  assert(rate > 0.0 && rate <= 1.0);
  for(size_t i=0;i<this->mirror.vmax.size();i++) {
    this->mirror.vmax[i] = this->mirror.vmax0[i]*rate;
    this->mirror.amax[i] = this->mirror.amax0[i]*rate;
    this->mirror.dmax[i] = this->mirror.dmax0[i]*rate;
  }
  if(!virtualController) {
    this->t.SetVelocityLimits(this->mirror.vmax);
    this->t.SetAccelerationLimits(this->mirror.amax);
    this->t.SetDecelerationLimits(this->mirror.dmax);
  }
}

bool SafeTrajClient::GetEndConfig(Vector& jend)
{
  if(sendqs.empty()) return mirror.GetEndConfig(jend);
  else {
    jend = sendqs.back();
    return true;
  }
}
bool SafeTrajClient::GetEndVelocity(Vector& vend)
{
  if(sendqs.empty()) return mirror.GetEndVelocity(vend);
  else {
    vend = sendvs.back();
    return true;
  }
}

double SafeTrajClient::GetEndTime()
{
  if(sendqs.empty()) return mirror.GetTrajEndTime();
  else {
    return sendts.back();
  }
}

SafeTrajClient::MotionResult SafeTrajClient::Brake(double tSystem,double dt)
{
  if(dt < 0) return InvalidParams;
  double t = EstimateCurrentTime(tSystem)+dt;
  Vector q;
  Vector dq;
  mirror.GetTrajConfig(t,q);
  mirror.GetTrajVelocity(t,dq);
  ParabolicRamp::ParabolicRampND ramp;
  ramp.x0 = q;
  ramp.dx0 = dq;
  Vector rdmax(mirror.dmax.size());
  for(size_t i=0;i<mirror.dmax.size();i++) 
    rdmax[i] = kAccSafetyMultiplier*mirror.dmax[i];
  ramp.SolveBraking(rdmax);
  printf("Braking at time %g\n",t);
  bool res=CheckTrajectory(t,Cast(ramp));
  assert(res);
  return this->SendTrajectory(t,Cast(ramp));
}

SafeTrajClient::MotionResult SafeTrajClient::BrakeAbs(double t)
{
  if(t < mirror.GetCurrentTime()) 
    return InvalidParams;
  printf("Braking at time %g, current time %g, mirror time %g\n",t,GetCurrentTime(),mirror.GetCurrentTime());
  Vector q;
  Vector dq;
  mirror.GetTrajConfig(t,q);
  mirror.GetTrajVelocity(t,dq);
  ParabolicRamp::ParabolicRampND ramp;
  ramp.x0 = q;
  ramp.dx0 = dq;
  Vector rdmax(mirror.dmax.size());
  for(size_t i=0;i<mirror.dmax.size();i++) 
    rdmax[i] = kAccSafetyMultiplier*mirror.dmax[i];
  ramp.SolveBraking(rdmax);
  printf("Brake start config: ");
  for(size_t i=0;i<q.size();i++)
    printf("%g ",q[i]);
  printf("\n");
  printf("Brake start velocity: ");
  for(size_t i=0;i<q.size();i++)
    printf("%g ",dq[i]);
  printf("\n");
  bool res=CheckTrajectory(t,Cast(ramp));
  assert(res);
  return this->SendTrajectory(t,Cast(ramp));
}

SafeTrajClient::MotionResult SafeTrajClient::ResetTrajectoryAbs(double t)
{
  if(mirror.GetTrajOffset(t) >= mirror.numSegments && sendIndex < (int)sendts.size()) {
    //still sending to motion queue -- can just cut off the send queue at the
    //right point
    sendReset = -1;
    double tprev = mirror.GetTrajEndTime();
    for(int i=sendIndex;i<(int)sendts.size();i++) {
      if(tprev + sendts[i] > t) {
	if(t-tprev > 0) {
	  //cut it off at index i
	  Vector priorConfig,priorVelocity;
	  if(i==sendIndex) {
	    mirror.GetEndConfig(priorConfig);
	    mirror.GetEndVelocity(priorVelocity);
	  }
	  else {
	    priorConfig=sendqs[i-1];
	    priorVelocity=sendvs[i-1];
	  }
	  Vector x,v;
	  HInterpolate(tprev,priorConfig,priorVelocity,sendts[i],sendqs[i],sendvs[i],t,x,v);
	  sendqs[i]=x;
	  sendvs[i]=v;
	  sendts[i]=t;
	  sendts.resize(i+1);
	  sendqs.resize(i+1);
	}
	else {
	  sendts.resize(i);
	  sendqs.resize(i);
	  sendvs.resize(i);
	}
        return Success;
      }
      tprev = sendts[i];
    }
    //past end of send queue -- just add another wait
    if(t - tprev > 0) {
      sendts.push_back(t);
      sendqs.push_back(sendqs.back());
      sendvs.push_back(sendvs.back());
    }
    return Success;
  }

  //clear send status
  sendts.clear();
  sendqs.clear();
  sendvs.clear();
  sendIndex = 0;
  sendReset = t;

  return Success;
}

SafeTrajClient::MotionResult SafeTrajClient::ResetTrajectoryRel(double dt)
{
  return ResetTrajectoryAbs(GetCurrentTime()+dt);
}

SafeTrajClient::MotionResult SafeTrajClient::MoveTo(const Vector& q,double rate,double dt)
{
  assert(rate > 0.0 and rate <= 1.0);
  if(!this->CheckState(q)) return FailedCheck;
        
  ParabolicRamp::ParabolicRampND ramp;
  GetEndConfig(ramp.x0);
  GetEndVelocity(ramp.dx0);
  for(size_t i=0;i<ramp.dx0.size();i++)
    if(fabs(ramp.dx0[i]) <= 0.004*mirror.dmax[i])
      ramp.dx0[i] = 0.0;
    else
      printf("MoveTo: Warning, end velocity %d is not zero\n",i);
  ramp.x1 = q;
  ramp.dx1.resize(q.size(),0.0);
  //scale accel/velocity limits
  Vector ramax,rvmax;
  double accRate = rate*kAccSafetyMultiplier;
  double velRate = rate*kVelSafetyMultiplier;
  ramax.resize(mirror.amax.size());
  rvmax.resize(mirror.vmax.size());
  for(size_t i=0;i<mirror.dmax.size();i++) {
    ramax[i] = accRate*Min(mirror.amax[i],mirror.dmax[i]);
    rvmax[i] = velRate*mirror.vmax[i];
  }
  //solve
  if (dt == 0.0) {
    bool res=ramp.SolveMinTimeLinear(ramax,rvmax);
    assert(res);
  }
  else {
    bool res=ramp.SolveMinAccelLinear(rvmax,dt);
    assert(res);
    Vector a;
    ramp.Accel(0,a);
    //check for acceleration feasibility
    for(size_t i=0;i<a.size();i++)
      if(fabs(a[i]) > mirror.amax[i]) {
	res=ramp.SolveMinTimeLinear(ramax,rvmax);
	assert(res);
	break;
      }
  }
  PiecewisePolynomialND poly=Cast(ramp);
  /*
  printf("Ramp end time %g\n",ramp.endTime);
  printf("Testing poly...\n");
  printf("%d elements\n",poly.elements.size());
  for(size_t i=0;i<poly.elements.size();i++) {
    printf("element %d\n",i);
    printf("%d segments, %d shift, %d times\n",poly.elements[i].segments.size(),poly.elements[i].timeShift.size(),poly.elements[i].times.size());
  }
  poly.Evaluate(0);
  poly.Evaluate(0.5);
  */
  bool res=CheckTrajectory(-1,poly);
  assert(res);
  return this->SendTrajectory(-1,poly);
}

SafeTrajClient::MotionResult SafeTrajClient::MoveToImmediate(double tSystem,double dt,const Vector& q,double rate)
{
  assert(rate > 0.0 and rate <= 1.0);
  if(!this->CheckState(q)) return FailedCheck;

  //scale accel/velocity limits
  Vector ramax,rvmax;
  double accRate = rate*kAccSafetyMultiplier;
  double velRate = rate*kVelSafetyMultiplier;
  ramax.resize(mirror.dmax.size());
  rvmax.resize(mirror.vmax.size());
  for(size_t i=0;i<mirror.dmax.size();i++) {
    ramax[i] = accRate*Min(mirror.amax[i],mirror.dmax[i]);
    rvmax[i] = velRate*mirror.vmax[i];
  }
        
  double t=EstimateCurrentTime(tSystem);
  ParabolicRamp::ParabolicRampND ramp;
  mirror.GetTrajConfig(t+dt,ramp.x0);
  mirror.GetTrajVelocity(t+dt,ramp.dx0);
  ramp.x1 = q;
  ramp.dx1.resize(q.size(),0.0);
  //solve
  //bool res=ramp.SolveMinTime(ramax,rvmax);
  //assert(res);
  vector<vector<ParabolicRamp::ParabolicRamp1D> > ramps;
  double endTime = SolveMinTimeBounded(ramp.x0,ramp.dx0,ramp.x1,ramp.dx1,
				     ramax,rvmax,mirror.qmin,mirror.qmax,
				     ramps);
  if(endTime < 0) {
    printf("Warning, ramp would cause robot to exceed joint limits?\n");
    return FailedCheck;
  }
  //bool res=CheckTrajectory(t+dt,Cast(ramps));
  //assert(res);
  return this->SendTrajectory(t+dt,Cast(ramps));
}

SafeTrajClient::MotionResult SafeTrajClient::MoveTraj(const std::vector<Vector>& q,const std::vector<Vector>& dq,const std::vector<double>& dt,double rate)
{
  //sanity check
  if(!dq.empty())
    assert(dq.size()==q.size());
  if(!dt.empty())
    assert(dt.size()==q.size());
  for(size_t i=0;i<q.size();i++) {
    assert(q[i].size()==mirror.qmin.size());
    if(!dq.empty())
      assert(dq[i].size()==mirror.qmin.size());
    if(!dt.empty()) 
      assert(dt[i] >= 0);
  }
 
  if(q.empty()) return InvalidParams; 

  //scale accel/velocity limits
  Vector ramax,rvmax,rdmax;
  double accRate = rate*kAccSafetyMultiplier;
  double velRate = rate*kVelSafetyMultiplier;
  ramax.resize(mirror.amax.size());
  rdmax.resize(mirror.amax.size());
  rvmax.resize(mirror.vmax.size());
  for(size_t i=0;i<mirror.amax.size();i++) {
    ramax[i] = accRate*mirror.amax[i];
    rdmax[i] = accRate*mirror.dmax[i];
    rvmax[i] = velRate*mirror.vmax[i];
  }

  /*
  vector<ParabolicRamp::ParabolicRampND> ramps(q.size());
  mirror.GetEndConfig(ramps[0].x0);
  mirror.GetEndVelocity(ramps[0].dx0);
  for(size_t i=0;i<q.size();i++) {
    ramps[i].x1=q[i];
    if(dq.empty()) 
      ramps[i].x1=ramps[0].dx0;
    else
      ramps[i].dx1=dq[i];
    
    if(dt.empty()) {
      bool res=ramps[i].SolveMinTime(ramax,rvmax);
      assert(res);
    }
    else {
      bool res=ramps[i].SolveMinAccel(rvmax,dt[i]);
      assert(res);
      //TODO: check accel
    }

    if(i+1 < q.size()) {
      ramps[i+1].x0 =ramps[i].x1;
      ramps[i+1].dx0 =ramps[i].dx1;
    }
  }
  
  if(!dq.empty()) {
    bool nonzero=false;
    for(size_t i=0;i<dq.back().size();i++)
      if(fabs(dq.back()[i]) != 0.0) nonzero = true;
    if(nonzero) {
      //final dq is nonzero, add a braking trajectory
      ramps.resize(ramps.size()+1);
      ramps.back().x0 = q.back();
      ramps.back().dx0 = dq.back();
      ramps.back().SolveBraking(rdmax);
    }
  }
  */
  vector<vector<vector<ParabolicRamp::ParabolicRamp1D> > > ramps(q.size());
  Vector xp,dxp;
  mirror.GetEndConfig(xp);
  mirror.GetEndVelocity(dxp);
  for(size_t i=0;i<q.size();i++) {
    Vector x = q[i];
    Vector dx;
    if(dq.empty()) 
      dx.resize(x.size(),0.0);
    else
      dx=dq[i];
    
    if(dt.empty()) {
      double endTime = SolveMinTimeBounded(xp,dxp,x,dx,ramax,rvmax,mirror.qmin,mirror.qmax,ramps[i]);
      assert(endTime >= 0);
    }
    else {
      bool res = SolveMinAccelBounded(xp,dxp,x,dx,dt[i],rvmax,mirror.qmin,mirror.qmax,ramps[i]);
      assert(res);
      //TODO: check accel
    }

    xp = x;
    dxp = dx;
  }
  
  if(!dq.empty()) {
    bool nonzero=false;
    for(size_t i=0;i<dq.back().size();i++)
      if(fabs(dq.back()[i]) != 0.0) nonzero = true;
    if(nonzero) {
      //final dq is nonzero, add a braking trajectory
      ramps.resize(ramps.size()+1);
      ParabolicRamp::ParabolicRampND ramp;
      ramp.x0 = q.back();
      ramp.dx0 = dq.back();
      ramp.SolveBraking(rdmax);
      ramps.back().resize(ramp.ramps.size());
      for(size_t i=0;i<ramp.ramps.size();i++) 
	ramps.back()[i].resize(1,ramp.ramps[i]);
    }
  }

  PiecewisePolynomialND poly;
  poly=Cast(ramps[0]);
  for(size_t i=1;i<ramps.size();i++)
    poly.Concat(Cast(ramps[i]),true);
  bool res=CheckTrajectory(-1,poly);
  assert(res);
  return SendTrajectory(-1,poly);
}

SafeTrajClient::MotionResult SafeTrajClient::MoveTraj(const std::vector<Vector>& q,const std::vector<Vector>& dq,double rate)
{
  vector<double> dt;
  return MoveTraj(q,dq,dt,rate);
}

SafeTrajClient::MotionResult SafeTrajClient::MoveTraj(const std::vector<Vector>& q,const std::vector<double>& dt,double rate)
{
  vector<Vector> dq;
  return MoveTraj(q,dq,dt,rate);
}

SafeTrajClient::MotionResult SafeTrajClient::MoveTraj(const std::vector<Vector>& q,double rate)
{
  vector<Vector> dq;
  vector<double> dt;
  return MoveTraj(q,dq,dt,rate);
}


bool SafeTrajClient::CheckState(const Vector& q) const
{
  Vector dq,ddq;
  return CheckState(q,dq,ddq);
}

bool SafeTrajClient::CheckState(const Vector& q,const Vector& dq) const
{
  Vector ddq;
  return CheckState(q,dq,ddq);
}

bool SafeTrajClient::CheckState(const Vector& q,const Vector& dq,const Vector& ddq) const
{
  assert(q.size() == mirror.qmin.size());
  double dt = 1.0/mirror.maxRate;
  for(size_t i=0;i<q.size();i++) {
    if(q[i] < mirror.qmin[i] || q[i] > mirror.qmax[i]) {
      fprintf(stderr,"CheckState: %g > q[%d]=%g > %g\n",mirror.qmin[i],i,q[i],mirror.qmax[i]);
      return false;
    }
    if(!dq.empty()) {
      if (fabs(dq[i]) > mirror.vmax[i]) {
	fprintf(stderr,"CheckState: Vel[%d]=%g > %g\n",i,dq[i],mirror.vmax[i]);
	return false;
      }
    }
    if(!ddq.empty()) {
      if(fabs(dq[i]+ddq[i]*dt) > fabs(dq[i])) { //accelerating
	if(fabs(ddq[i]) > mirror.amax[i]) {
	  fprintf(stderr,"CheckState: Acc[%d]=%g > %g\n",i,ddq[i],mirror.amax[i]);
	  return false;
	}
      }
      else {
	if(fabs(ddq[i]) > mirror.dmax[i]) {
	  fprintf(stderr,"CheckState: Dec[%d]=%g > %g\n",i,ddq[i],mirror.dmax[i]);
	  return false;
	}
      }
    }
  }
  return true;
}



void SafeTrajClient::FlushSendQueue()
{
  if(sendIndex >= (int)sendqs.size()) return;
  /*
  fprintf(stderr,"FlushSendQueue: TEMP: fix me\n");
  abort();
  */
  while (sendIndex < (int)sendqs.size()) {
    //TEMP: do we need system time
    double sleept = SendPoll(0);
    if(sleept > 0) {
      printf("Sleeping for time %g\n",sleept);
      ThreadSleep(sleept);
    }
  }
}

double SafeTrajClient::SyncTimers(double tSystem)
{
  //make sure the mirror is sync'ed in time
  double t=this->t.GetCurrentTime();
  if(t > this->mirror.currentTime)
    this->mirror.AdvanceTime(t,tSystem);
  else {
    printf("SyncTimers: Warning, time is %g, mirror time is %g\n",t,this->mirror.currentTime);
    this->mirror.currentTime = t;
    this->mirror.currentTimeUpdateTime = tSystem;
  }
  return t;
}

double SafeTrajClient::SendPoll(double tSystem)
{
  if(sendIndex == (int)sendqs.size()) return 0;

  //printf("SyncTimers...\n");
  Timer timer;
  //SyncTimers(tSystem);

  //printf("SyncTimers time %g\n",timer.ElapsedTime());
  bool done = DoSendMilestones();
  //printf("Send time %g\n",timer.ElapsedTime());

  if(done) {
    //printf("Done sendPoll\n");
    return 0;
  }
  else { //not done yet, request a sleep
    //printf("Current # of segments: %d, approx %g seconds left to send\n",mirror.GetCurSegments(),this->mirror.GetTrajEndTime()-this->mirror.GetCurrentTime());
    //this is a good amount to sleep
    return 0.5*(this->mirror.GetTrajEndTime()-this->mirror.GetCurrentTime());
  }
}

bool SafeTrajClient::DoSendMilestones()
{
  if(sendReset >= 0) 
    this->mirror.ResetTrajectoryAbs(sendReset);
  if(sendIndex == (int)sendqs.size()) {
    if(sendReset >= 0) {
      if(!virtualController) {
	bool res=this->t.ResetTrajectoryAbs(sendReset);
	//clear sendReset
	if(!res) {
	  if(t.flog)
	    t.flog<<AscLocalTime()<<": SafeTrajClient::DoSendMilestones: resetTrajectory("<<sendReset<<") failed"<<endl;
	  else
	    printf("SafeTrajClient::DoSendMilestones: resetTrajectory(%g) failed\n",sendReset);
	  sendReset = -1;
	  return false;
	}
	sendReset = -1;
      }
    }
    return true;
  }
  assert(sendqs.size()==sendts.size());
  int cs = mirror.GetCurSegments();
  if(GetMaxSegments()-cs < 20)
    printf("SafeTrajClient::DoSendMilestones: Sending: %d, available: %d\n",cs,GetMaxSegments()-cs);
  while(cs < GetMaxSegments()) {
    //printf("Current segments: %d, remaining %d\n",cs,GetMaxSegments()-cs);
    int lim = std::min(milestoneBatch,GetMaxSegments()-cs);
    int nmax = std::min(sendIndex+lim,(int)sendqs.size());
    //printf("Sending %d milestones\n",nmax-sendIndex);
    std::vector<double> tslice(sendts.begin()+sendIndex,sendts.begin()+nmax);
    std::vector<Vector> qslice(sendqs.begin()+sendIndex,sendqs.begin()+nmax);
    std::vector<Vector> vslice(sendvs.begin()+sendIndex,sendvs.begin()+nmax);
    assert(tslice.size()==nmax-sendIndex);
    if(!virtualController) {
      if(sendReset >= 0) {
	bool res=this->t.AppendMilestonesImmediate(sendReset,tslice,qslice,vslice);
	assert(res);
	//clear sendReset
	sendReset = -1;
      }
      else {
	bool res=this->t.AppendMilestonesQuiet(tslice,qslice,vslice);
	assert(res);
      }
    }
    bool res=this->mirror.AppendMilestonesQuiet(tslice,qslice,vslice);
    assert(res);
    cs += (int)tslice.size();

    sendIndex = nmax;
    if(sendIndex == (int)sendqs.size()) {
      return true;
    }
  }
  //not done yet, wait
  return false;
}



SafeTrajClient::MotionResult SafeTrajClient::AddTrajectory(const Spline::PiecewisePolynomialND& f)
{
  return AddTrajectoryImmediate(-1,f);
}

SafeTrajClient::MotionResult SafeTrajClient::AddTrajectory(const std::vector<double>& dt,const std::vector<Vector>& q,const std::vector<Vector>& v)
{
  return AddTrajectoryImmediate(-1,dt,q,v);
}

SafeTrajClient::MotionResult SafeTrajClient::AddTrajectoryImmediate(double tstart,const Spline::PiecewisePolynomialND& f)
{
  if(!this->CheckTrajectory(tstart,f)) return FailedCheck;
  return this->SendTrajectory(tstart,f);
}

SafeTrajClient::MotionResult SafeTrajClient::AddTrajectoryImmediate(double tstart,const std::vector<double>& dt,const std::vector<Vector>& q,const std::vector<Vector>& v)
{
  if(!this->CheckTrajectory(tstart,dt,q,v)) return FailedCheck;
  return this->SendTrajectory(tstart,dt,q,v);
}

bool SafeTrajClient::CheckTrajectory(double tstart,const Spline::PiecewisePolynomialND& f) 
{
  vector<double> t;
  vector<Vector> q,v;
  DiscretizeTrajectory(f,t,q,v);
  return CheckTrajectory(tstart,t,q,v);
}

bool SafeTrajClient::CheckTrajectory(double tstart,const std::vector<double>& t,const std::vector<Vector>& q,const std::vector<Vector>& v) 
{
  Vector qprev;
  Vector dqprev;
  double tinit;
  double tprev;
  if(tstart < 0) {  //add to end of motion queue
    GetEndConfig(qprev);
    GetEndVelocity(dqprev);
    tprev=GetEndTime();
    tprev = Max(tprev,mirror.GetCurrentTime()+2.0*GetAverageLatency());
  }
  else {
    if(tstart < GetCurrentTime()) {
      fprintf(stderr,"SafeTrajClient::CheckPath: reset is before current time\n");
      return false;
    }
    if(tstart > mirror.GetTrajEndTime() && sendIndex < (int)sendts.size()) {
      fprintf(stderr,"SafeTrajClient::CheckPath: reset is after current segments, and I'm still sending\n");
      abort();
      return false;
    }
    mirror.GetTrajConfig(tstart,qprev);
    mirror.GetTrajVelocity(tstart,dqprev);
    tprev = tstart;
  }
  tinit=tprev;
  if(t[0] == 0) {
    for(size_t i=0;i<q[0].size();i++) {
      if(fabs(q[0][i]-qprev[i]) > 1e-3) {
	fprintf(stderr,"SafeTrajClient::CheckPath: start config differs by more than 1e-3: %g vs %g\n",q[0][i],qprev[i]);
	return false;
      }
      if(fabs(v[0][i]-dqprev[i]) > 1e-2) {
	fprintf(stderr,"SafeTrajClient::CheckPath: start velocity differs by more than 1e-2: %g vs %g\n",v[0][i],dqprev[i]);
	return false;
      }
    }
  }
  for(size_t i=0;i<q.size();i++) {
    if(!CheckHermite(tprev,qprev,dqprev,t[i]+tinit,q[i],v[i],
		     mirror.qmin,mirror.qmax,mirror.vmax,mirror.amax)) {
      fprintf(stderr,"SafeTrajClient::CheckPath: Segment %d failed hermite path check\n",i);
      return false;
    }
    qprev=q[i];
    dqprev=v[i];
    tprev=t[i]+tinit;
  }
  return true;
}

SafeTrajClient::MotionResult SafeTrajClient::SendTrajectory(double tstart,const Spline::PiecewisePolynomialND& f)
{
  std::vector<double> t;
  std::vector<Vector> q,v;
  DiscretizeTrajectory(f,t,q,v);
  return SendTrajectory(tstart,t,q,v);
}

SafeTrajClient::MotionResult SafeTrajClient::SendTrajectory(double tstart,const std::vector<double>& t,const std::vector<Vector>& q,const std::vector<Vector>& v)
{
  //Timer timer;
  double toffset;
  if(tstart >= 0) {
    //reset the trajectory starting from tstart
    if(ResetTrajectoryAbs(tstart) != Success) {
      if(this->t.flog)
	this->t.flog<<AscLocalTime()<<": SafeTrajClient::SendTrajectory: ResetTrajectoryAbs("<<tstart<<") failed"<<endl;
      else
	fprintf(stderr,"ResetTrajectoryAbs(%g) failed\n",tstart);
      return TransmitError;
    }
    toffset=tstart;
    //printf("Time for resetTrajectory: %g\n",timer.ElapsedTime());
  }
  else {
    toffset = GetEndTime();
    toffset = Max(toffset,mirror.GetCurrentTime()+2.0*GetAverageLatency());
    if(this->t.flog)
      this->t.flog<<AscLocalTime()<<": SafeTrajClient::SendTrajectory: Automatically setting start time "<<toffset<<", current "<<mirror.GetCurrentTime()<<endl;
    else
      printf("SendTrajectory: Automatically setting start time %g, current %g\n",toffset,mirror.GetCurrentTime());
  }

  sendqs.erase(sendqs.begin(),sendqs.begin()+sendIndex);
  sendvs.erase(sendvs.begin(),sendvs.begin()+sendIndex);
  sendts.erase(sendts.begin(),sendts.begin()+sendIndex);
  sendIndex = 0;
  size_t ignoreFirst = 0;
  if(!t.empty() && t[0] == 0)
    ignoreFirst = 1;
  sendqs.insert(sendqs.end(),q.begin()+ignoreFirst,q.end());
  sendvs.insert(sendvs.end(),v.begin()+ignoreFirst,v.end());
  //sendts.insert(sendts.end(),t.begin(),t.end());
  for(size_t i=ignoreFirst;i<t.size();i++)
    sendts.push_back(t[i]+toffset);
      
  if(tstart != 0) {
    //send an immediate batch of milestones
    bool res=DoSendMilestones();
    if(!res) {
      if(this->t.flog)
	this->t.flog<<AscLocalTime()<<": SafeTrajClient::SendTrajectory: Warning, could not immediately send all milestones... will require continued sending..."<<endl;
      else
	printf("SafeTrajClient::SendTrajectory: Warning, could not immediately send all milestones... will require continued sending...\n");
    }
    //assert(res);
    //printf("Send immediate batch result %d\n",(int)res);
    //printf("Time for reset + sendMilestones: %g\n",timer.ElapsedTime());
    //t.GetCurrentTime();
    //printf("Time for becoming responsive: %g\n",timer.ElapsedTime());
  }

  //if not nonblocking, wait until the trajectory is sent
  if(!nonblock) {
    FlushSendQueue();
  } 
  return Success;
}
