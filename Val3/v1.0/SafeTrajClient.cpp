#include "SafeTrajClient.h"
#include <spline/PiecewisePolynomial.h>
#include "Modeling/ParabolicRamp.h"
#include "Modeling/Conversions.h"
#include "Timer.h"
#include <assert.h>
using namespace std;
using namespace Spline;

double kVelSafetyMultiplier = 0.99;
double kAccSafetyMultiplier = 0.99;


SafeTrajClient::SafeTrajClient(const char* host,int port)
  :milestoneBatch(100),nonblock(false),virtualController(false),sendIndex(0)
{
  if(host) {
    bool res=Connect(host,port);
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

  sendIndex = 0;
  sendqs.clear();
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
  Timer timer;
  for (int i=0;i<iters;i++)
    this->t.Echo("");
  this->latency = timer.ElapsedTime()/iters;
}

void SafeTrajClient::SetNonblocking(bool nonblock)
{
  FlushSendQueue();
  this->nonblock = nonblock;
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
    if(sendqs.size()==1)
      mirror.GetEndConfig(vend);
    else
      vend = sendqs[sendqs.size()-2];
    for(size_t i=0;i<vend.size();i++)
      vend[i] = (sendqs.back()[i]-vend[i])/sendts[i];
    return true;
  }
}

bool SafeTrajClient::Brake(double tSystem,double dt)
{
  if(dt < 0) return false;
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
  bool res=CheckTrajectory(t,Cast(ramp),ramp.endTime);
  assert(res);
  return this->SendTrajectory(t,Cast(ramp),ramp.endTime);
}

bool SafeTrajClient::BrakeAbs(double t)
{
  if(t < mirror.GetCurrentTime()) 
    return false;
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
  bool res=CheckTrajectory(t,Cast(ramp),ramp.endTime);
  assert(res);
  return this->SendTrajectory(t,Cast(ramp),ramp.endTime);
}

bool SafeTrajClient::ResetTrajectoryAbs(double t)
{
  if(mirror.GetTrajOffset(t) >= mirror.numSegments && sendIndex < (int)sendts.size()) {
    //still sending to motion queue -- can just cut off the send queue at the
    //right point
    double tend = mirror.GetTrajEndTime();
    for(int i=sendIndex;i<(int)sendts.size();i++) {
      if(tend + sendts[i] > t) {
	if(t-tend > 0) {
	  //cut it off at index i
	  double u=(t-tend)/sendts[i];
	  Vector priorConfig;
	  if(i==sendIndex) mirror.GetEndConfig(priorConfig);
	  else priorConfig=sendqs[i-1];
	  for(size_t j=0;j<sendqs[i].size();j++)
	    sendqs[i][j] = priorConfig[j] + u*(sendqs[i][j]-priorConfig[j]);
	  sendts[i] = t-tend;
	  sendts.resize(i+1);
	  sendqs.resize(i+1);
	}
	else {
	  sendts.resize(i);
	  sendqs.resize(i);
	}
        return true;
      }
      tend += sendts[i];
    }
    //past end of send queue -- just add another wait
    if(t - tend > 0) {
      sendts.push_back(t-tend);
      sendqs.push_back(sendqs.back());
    }
    return true;
  }

  //clear send status
  sendts.clear();
  sendqs.clear();
  sendIndex = 0;

  if(!virtualController) {
    bool res=this->t.ResetTrajectoryAbs(t);
    if(!res) return false;
  }

  this->mirror.ResetTrajectoryAbs(t);
  return true;
}

bool SafeTrajClient::ResetTrajectoryRel(double dt)
{
  return ResetTrajectoryAbs(GetCurrentTime()+dt);
}

bool SafeTrajClient::MoveTo(const Vector& q,double rate,double dt)
{
  assert(rate > 0.0 and rate <= 1.0);
  this->CheckState(q);
        
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
  bool res=CheckTrajectory(0,poly,poly.EndTime());
  assert(res);
  return this->SendTrajectory(0,poly,ramp.endTime);
}

bool SafeTrajClient::MoveToImmediate(double tSystem,double dt,const Vector& q,double rate)
{
  assert(rate > 0.0 and rate <= 1.0);
  this->CheckState(q);

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
    return false;
  }
  //bool res=CheckTrajectory(t+dt,Cast(ramps),endTime);
  //assert(res);
  return this->SendTrajectory(t+dt,Cast(ramps),endTime);
}

bool SafeTrajClient::MoveTraj(const std::vector<Vector>& q,const std::vector<Vector>& dq,const std::vector<double>& dt,double rate)
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
 
  if(q.empty()) return true;       

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
  bool res=CheckTrajectory(0,poly,poly.EndTime());
  assert(res);
  return SendTrajectory(0,poly,poly.EndTime());
}

bool SafeTrajClient::MoveTraj(const std::vector<Vector>& q,const std::vector<Vector>& dq,double rate)
{
  vector<double> dt;
  return MoveTraj(q,dq,dt,rate);
}

bool SafeTrajClient::MoveTraj(const std::vector<Vector>& q,const std::vector<double>& dt,double rate)
{
  vector<Vector> dq;
  return MoveTraj(q,dq,dt,rate);
}

bool SafeTrajClient::MoveTraj(const std::vector<Vector>& q,double rate)
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
  fprintf(stderr,"TEMP: fix me\n");
  abort();
  /*
  while (sendIndex < (int)sendqs.size()) {
    double sleept = SendPoll();
    if(sleept > 0) 
      usleep(int(sleept*1000000));
  }
  */
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
  printf("Send time %g\n",timer.ElapsedTime());

  double tserver=this->GetTrajEndTime();	
  double tmirror=mirror.GetTrajEndTime();	
  //clock skew comes from AdvanceTime, sending overhead, and network send delays
  printf("Mirror clock skew: %g\n",tmirror-tserver);
  mirror.currentTime += tserver - tmirror;
  for(int i=0;i<=mirror.numSegments;i++)
    mirror.GetTrajTime(mirror.curSegment + i) += tserver-tmirror;

  //double dt=timer.ElapsedTime();
  //mirror.AdvanceTime(mirror.currentTime+dt,mirror.currentTimeUpdateTime+dt);
  printf("Send and process time %g\n",timer.ElapsedTime());

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
  if(sendIndex == (int)sendqs.size()) return true;
  assert(sendqs.size()==sendts.size());
  int cs = mirror.GetCurSegments();
  while(cs < GetMaxSegments()) {
    //printf("Current segments: %d, remaining %d\n",cs,GetMaxSegments()-cs);
    int lim = std::min(milestoneBatch,GetMaxSegments()-cs);
    int nmax = std::min(sendIndex+lim,(int)sendqs.size());
    //printf("Sending %d milestones\n",nmax-sendIndex);
    std::vector<double> tslice(sendts.begin()+sendIndex,sendts.begin()+nmax);
    std::vector<Vector> qslice(sendqs.begin()+sendIndex,sendqs.begin()+nmax);
    assert(tslice.size()==nmax-sendIndex);
    if(!virtualController) {
      bool res=this->t.AppendMilestonesQuiet(tslice,qslice);
      assert(res);
    }
    bool res=this->mirror.AppendMilestonesQuiet(tslice,qslice);
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
