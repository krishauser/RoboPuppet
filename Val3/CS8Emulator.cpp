#include "CS8Emulator.h"
#include <utils/threadutils.h>
#include <tinyxml.h>
#include <assert.h>
#include <math.h>
#include <algorithm>
using namespace std;

const static bool kCheckNonzeroVelocity = true;
const static double kServoTimestep = 0.004;

//at 0, there is no simulated velocity damping
//at 1, the velocity damps to 0 in a second
//const static double kServoDampingRatioPerSecond = 0;
const static double kServoDampingRatioPerSecond = 0.5;
//1-r = (1-rs)^(1/dt)
//(1-r)^dt = (1-rs)

inline double Sign(double x) { if(x < 0) return -1.0; else if(x > 0) return 1.0; else return 0.0; }

//hermite interpolation
void HInterpolate(double t1,double q1,double v1,
		  double t2,double q2,double v2,
		  double t,double& qt,double& dqt)
{
  double dt=(t2-t1);
  double u = (t-t1)/dt;
  double u2 = u*u;
  double u3 = u*u*u;
  double cq1 = 2.0*u3-3.0*u2+1;
  double cq2 = -2.0*u3+3.0*u2;
  double cv1 = dt*(u3-2.0*u2+u);
  double cv2 = dt*(u3-u2);
  double dcq1 = (6.0*u2-6.0*u)/dt;
  double dcq2 = (-6.0*u2+6.0*u)/dt;
  double dcv1 = 3.0*u2-4.0*u+1.0;
  double dcv2 = 3.0*u2-2.0*u;
  qt = cq1*q1 + cq2*q2 + cv1*v1 + cv2*v2;
  dqt = dcq1*q1 + dcq2*q2 + dcv1*v1 + dcv2*v2;
}

//hermite interpolation
void HInterpolate(double t1,const CS8Emulator::Vector& q1,const CS8Emulator::Vector& v1,
		  double t2,const CS8Emulator::Vector& q2,const CS8Emulator::Vector& v2,
		  double t,CS8Emulator::Vector& qt,CS8Emulator::Vector& dqt)
{
  if(t < t1 || t > t2) {
    printf("HInterpolate: fatal error: time is outside of valid range: %g not in [%g,%g]\n",t,t1,t2);
  }
  assert(t2 >= t && t >= t1);
  assert(q2.size()==q1.size());
  assert(v1.size()==q1.size());
  assert(v2.size()==q1.size());
  qt.resize(q1.size());
  dqt.resize(q1.size());
  double dt=(t2-t1);
  double u = (t-t1)/dt;
  double u2 = u*u;
  double u3 = u*u*u;
  double cq1 = 2.0*u3-3.0*u2+1.0;
  double cq2 = -2.0*u3+3.0*u2;
  double cv1 = dt*(u3-2.0*u2+u);
  double cv2 = dt*(u3-u2);
  double dcq1 = (6.0*u2-6.0*u)/dt;
  double dcq2 = (-6.0*u2+6.0*u)/dt;
  double dcv1 = 3.0*u2-4.0*u+1.0;
  double dcv2 = 3.0*u2-2.0*u;
  for(size_t i=0;i<q1.size();i++) {
    qt[i] = cq1*q1[i] + cq2*q2[i] + cv1*v1[i] + cv2*v2[i];
    dqt[i] = dcq1*q1[i] + dcq2*q2[i] + dcv1*v1[i] + dcv2*v2[i];
  }
}



CS8Emulator::CS8Emulator()
  :curSegment(0),numSegments(0),motionError(false)
{}

void CS8Emulator::Disconnect()
{
  milestones.clear();
  tangents.clear();
  times.clear();
  curSegment = 0;
  numSegments = 0;
  currentTime = 0;
  lastServoTime = 0;
  motionError = false;
}

bool CS8Emulator::Sync(TrajClient& t)
{
  bool res;
  res=t.GetJointLimits(qmin,qmax);
  assert(res);
  res=t.GetVelocityLimits(vmax);
  assert(res);
  res=t.GetAccelerationLimits(amax);
  assert(res);
  res=t.GetDecelerationLimits(dmax);
  assert(res);
  qmin0=qmin;
  qmax0=qmax;
  vmax0=vmax;
  amax0=amax;
  dmax0=dmax;
  maxSegments = t.GetMaxSegments();
  //HACK!
  maxSegments = 99;
  
  maxRate = t.Rate();

  string status = t.Status();
  while(status != "") {
    if(status == "paused") {
      printf("CS8Emulator: cannot sync to a paused trajectory server.\n");
      printf("   waiting for client to be unpaused...\n");
      ThreadSleep(2.0);
      status = t.Status();
    }
    else if(status == "motionError") {
      printf("CS8Emulator: trajectory server braked, resetting error.\n");
      while(t.GetTrajEndTime() > t.GetCurrentTime()) {
	printf("CS8Emulator: Waiting %gs until the motion stops...\n",t.GetTrajEndTime()-t.GetCurrentTime());
	ThreadSleep(2.0*(t.GetTrajEndTime()-t.GetCurrentTime()));
      }
      t.ClearError();
      status = t.Status();
      if(status == "motionError") {
	printf("CS8Emulator: ClearError not working\n");
	abort();
      }
      printf("CS8Emulator: trajectory server reset.\n");
    }
    else if(status != ""){
      printf("CS8Emulator: Status is something odd: \"%s\"\n",status.c_str());
      abort();
    }
  }
  motionError = false;

  milestones.resize(maxSegments+1);
  tangents.resize(maxSegments+1);
  times.resize(maxSegments+1);
  curSegment = 0;
  numSegments = 0;
  while(t.GetTrajEndTime() > t.GetCurrentTime()) {
    printf("CS8Emulator: Waiting %gs until the motion stops...\n",t.GetTrajEndTime()-t.GetCurrentTime());
    ThreadSleep(2.0*(t.GetTrajEndTime()-t.GetCurrentTime()));
  }
  //get current status
  res = t.GetEndConfig(qCurrent);
  assert(res);
  milestones[0] = qCurrent;
  dqCurrent.resize(qCurrent.size());
  fill(dqCurrent.begin(),dqCurrent.end(),0.0);
  qDesired = qEstimate = qCurrent;
  dqDesired = dqCommand = dqCurrent;
  tangents[0] = dqCurrent;
  times[0] = t.GetTrajEndTime();
  currentTime = t.GetCurrentTime();
  lastServoTime = currentTime;
  return true;
}

bool CS8Emulator::Setup(const char* xmlFile)
{
  Vector q(6,0.0);
  qmin.resize(6);
  qmax.resize(6);
  vmax.resize(6);
  amax.resize(6);
  dmax.resize(6);
  if(!xmlFile) {
    fill(qmin.begin(),qmin.end(),-45.0);
    fill(qmax.begin(),qmax.end(),45.0);
    fill(vmax.begin(),vmax.end(),20.0);
    fill(amax.begin(),amax.end(),50.0);
    fill(dmax.begin(),dmax.end(),50.0);
    maxSegments = 3000;  
    maxRate = 250.0;  //same as Staubli CS8
  }
  else {
    TiXmlDocument doc;
    if(!doc.LoadFile(xmlFile)) {
      fprintf(stderr,"Error reading XML file %s\n",xmlFile);
      return false;
    }
    if(0!=strcmp(doc.RootElement()->Value(),"val3specs")) {
      fprintf(stderr,"Wrong type of root element %s\n",doc.RootElement()->Value());
      return false;
    }
    TiXmlElement* e;
    e=doc.RootElement()->FirstChildElement("rate");
    if(e && e->QueryValueAttribute("value",&maxRate) != TIXML_SUCCESS) {
      fprintf(stderr,"Could not read value attribute of rate\n");
      maxRate = 250.0;
    }
    e=doc.RootElement()->FirstChildElement("queue");
    if(e && e->QueryValueAttribute("size",&maxSegments) != TIXML_SUCCESS) {
      fprintf(stderr,"Could not read size attribute of queue\n");
      maxSegments = 3000;
    }
    e=doc.RootElement()->FirstChildElement("joints");
    if(e) {
      int n;
      if(e->QueryValueAttribute("num",&n) == TIXML_SUCCESS) {
	q.resize(n);
	qmin.resize(n);
	qmax.resize(n);
	vmax.resize(n);
	amax.resize(n);
	dmax.resize(n);
      }
      if(e->Attribute("value")) {
	stringstream ss(e->Attribute("value"));
	for(size_t i=0;i<q.size();i++)
	  ss >> q[i];
      }
      if(e->Attribute("min")) {
	stringstream ss(e->Attribute("min"));
	for(size_t i=0;i<q.size();i++)
	  ss >> qmin[i];
      }
      if(e->Attribute("max")) {
	stringstream ss(e->Attribute("max"));
	for(size_t i=0;i<q.size();i++)
	  ss >> qmax[i];
      }
    }
    e=doc.RootElement()->FirstChildElement("velocity");
    if(e) {
      if(e->Attribute("max")) {
	stringstream ss(e->Attribute("max"));
	for(size_t i=0;i<q.size();i++)
	  ss >> vmax[i];
      }
    }
    e=doc.RootElement()->FirstChildElement("acceleration");
    if(e) {
      if(e->Attribute("max")) {
	stringstream ss(e->Attribute("max"));
	for(size_t i=0;i<q.size();i++)
	  ss >> amax[i];
      }
      if(e->Attribute("min")) {
	stringstream ss(e->Attribute("min"));
	for(size_t i=0;i<q.size();i++) {
	  ss >> dmax[i];
	  dmax[i] = fabs(dmax[i]);
	}
      }
      else
	dmax = amax;
    }
  }
  qmin0=qmin;
  qmax0=qmax;
  vmax0=vmax;
  amax0=amax;
  dmax0=dmax;

  milestones.resize(maxSegments+1);
  tangents.resize(maxSegments+1);
  times.resize(maxSegments+1);
  curSegment = 0;
  numSegments = 0;
  motionError = false;
  //get current status
  milestones[0] = q;
  qCurrent = q;
  dqCurrent.resize(qCurrent.size());
  fill(dqCurrent.begin(),dqCurrent.end(),0.0);
  qDesired = qEstimate = qCurrent;
  dqDesired = dqCommand = dqCurrent;
  tangents[0] = dqCurrent;
  times[0] = 0;
  currentTime = 0;
  lastServoTime = currentTime;
  return true;
}




std::string CS8Emulator::Status() const
{
  if(motionError) return "motionError";
  return "";
}

bool CS8Emulator::GetConfig(Vector& q) const
{
  q = qCurrent;
  return true;
  /*
  double t=currentTime;
  GetTrajConfig(t,q);
  return true;
  */
}

bool CS8Emulator::GetVelocity(Vector& v) const
{
  v = dqCurrent;
  return true;
  /*
  double t=currentTime;
  GetTrajVelocity(t,v);
  return true;
  */
}

bool CS8Emulator::GetTransform(Vector& x) const
{
  double t=currentTime;
  Vector q;
  GetTrajConfig(t,q);
  fprintf(stderr,"Virtual GetTransform not implemented yet\n");
  abort();
  return true;
}

bool CS8Emulator::GetJointLimits(Vector& jmin,Vector& jmax) const
{
  jmin = qmin;
  jmax = qmax;
  return true;
}

bool CS8Emulator::GetVelocityLimits(Vector& vmax) const
{
  vmax = this->vmax;
  return true;
}

bool CS8Emulator::GetAccelerationLimits(Vector& amax) const
{
  amax = this->amax;
  return true;
}

bool CS8Emulator::GetDecelerationLimits(Vector& dmax) const
{
  dmax = this->dmax;
  return true;
}

int CS8Emulator::GetMaxSegments() const
{
  return this->maxSegments;
}

int CS8Emulator::GetCurSegments() const
{
  return numSegments;
}

int CS8Emulator::GetRemainingSegments() const
{
  return GetMaxSegments()-GetCurSegments();
}

double CS8Emulator::GetCurrentTime() const
{
  return currentTime;
}

double CS8Emulator::GetTrajEndTime() const
{
  if(numSegments == 0) return currentTime;
  return GetTrajTime(curSegment+numSegments);
}


double CS8Emulator::GetTrajDuration() const
{
  return GetTrajEndTime()-GetCurrentTime();
}

bool CS8Emulator::GetEndConfig(Vector& jend) const
{
  jend = GetTrajMilestone(curSegment+numSegments);
  return true;
}

bool CS8Emulator::GetEndVelocity(Vector& vend) const
{
  if(numSegments == 0 || GetCurrentTime() > GetTrajTime(curSegment+numSegments)) {
    vend.resize(GetTrajMilestone(curSegment).size());
    fill(vend.begin(),vend.end(),0.0);
  }
  else {
    Vector x;
    EvalTrajSegment(numSegments-1,GetTrajTime(curSegment),x,vend);
  }
  return true;
}

bool CS8Emulator::CheckTrajectory()
{
  //TODO: actually check it
  return true;
}

bool CS8Emulator::SetJointLimits(const Vector& qmin,const Vector& qmax)
{
  assert(qmin.size()==this->qmin.size());
  assert(qmax.size()==this->qmax.size());
  for(size_t i=0;i<qmin.size();i++) {
    this->qmin[i] = std::max(this->qmin0[i],qmin[i]);
    this->qmax[i] = std::min(this->qmax0[i],qmax[i]);
    assert(this->qmax[i] >= this->qmin[i]);
  }
  return true;
}

bool CS8Emulator::SetVelocityLimits(const Vector& vmax)
{
  assert(vmax.size()==this->vmax.size());
  for(size_t i=0;i<vmax.size();i++) {
    assert(vmax[i] >= 0);
    this->vmax[i] = std::min(this->vmax0[i],vmax[i]);
  }
  return true;
}

bool CS8Emulator::SetAccelerationLimits(const Vector& amax)
{
  assert(amax.size()==this->amax.size());
  for(size_t i=0;i<amax.size();i++) {
    assert(amax[i] >= 0);
    this->amax[i] = std::min(this->amax0[i],amax[i]);
  }
  return true;
}

bool CS8Emulator::SetDecelerationLimits(const Vector& dmax)
{
  assert(amax.size()==this->amax.size());
  for(size_t i=0;i<amax.size();i++) {
    assert(dmax[i] >= 0);
    this->dmax[i] = std::min(this->dmax0[i],dmax[i]);
  }
  return true;
}

int CS8Emulator::GetTrajSegment(double t) const
{
  return curSegment + GetTrajOffset(t);
    /*
  assert(numSegments <= maxSegments);
  if(t <= GetTrajTime(curSegment)) return curSegment;
  else if(t >= GetTrajTime(curSegment+numSegments)) return curSegment+numSegments;
  else {
    int i=curSegment;
    while(t > GetTrajTime(i+1)) {
      i++;
    }
    return i;
  }
    */
}

int CS8Emulator::GetTrajOffset(double t) const
{
  assert(numSegments <= maxSegments);
  if(t <= GetTrajTime(curSegment)) return 0;
  else if(t >= GetTrajTime(curSegment+numSegments)) return numSegments;
  else {
    int begin=curSegment%(maxSegments+1);
    int end = (curSegment+numSegments)%(maxSegments+1);
    if(begin > end) {
      // do two searches because of the cyclic thing
      int ofs;
      if(t > times.back()) {
	vector<double>::const_iterator it = --std::upper_bound(times.begin(),times.begin()+end,t);
	ofs = numSegments-((times.begin()+end)-it);
      }
      else {
	vector<double>::const_iterator it = --std::upper_bound(times.begin()+begin,times.end(),t);
	ofs = it-(times.begin()+begin);
      }
      assert(t >= GetTrajTime(curSegment+ofs) && t <= GetTrajTime(curSegment+ofs+1));
      return ofs;
    }
    else {
      vector<double>::const_iterator it = --std::upper_bound(times.begin()+begin,times.begin()+end,t);
      int ofs = it-(times.begin()+begin);
      //printf("range %g %g, t=%g\n",GetTrajTime(curSegment+ofs),GetTrajTime(curSegment+ofs+1),t);
      assert(t >= GetTrajTime(curSegment+ofs) && t <= GetTrajTime(curSegment+ofs+1));
      return ofs;
    }
    /*
    int i=0;
    while(t > GetTrajTime(curSegment+i+1)) {
      i++;
    }
    return i;
    */
  }
}

void CS8Emulator::EvalTrajSegment(int i,double t,Vector& x,Vector& v) const
{
  if(i < 0) {
    x=GetTrajMilestone(curSegment);
    v.resize(x.size());
    fill(v.begin(),v.end(),0.0);
  }
  else if(i >= numSegments) {
    x=GetTrajMilestone(curSegment+numSegments); 
    v.resize(x.size());
    fill(v.begin(),v.end(),0.0);
  }
  else {
    i += curSegment;
    const Vector& m1 = GetTrajMilestone(i);
    const Vector& m2 = GetTrajMilestone(i+1);
    const Vector& v1 = GetTrajTangent(i);
    const Vector& v2 = GetTrajTangent(i+1);
    double t1=GetTrajTime(i);
    double t2=GetTrajTime(i+1);
    HInterpolate(t1,m1,v1,
		 t2,m2,v2,
		 t,x,v);
  }
}

void CS8Emulator::GetTrajConfig(double t,Vector& x) const
{
  int i=GetTrajOffset(t);
  Vector v;
  EvalTrajSegment(i,t,x,v);
}

void CS8Emulator::GetTrajVelocity(double t,Vector& v) const
{
  int i=GetTrajOffset(t);
  Vector x;
  EvalTrajSegment(i,t,x,v);

}

bool CS8Emulator::ResetTrajectoryAbs(double t)
{
  if(motionError) { printf("CS8Emulator: attempted to reset trajectory after motion error\n"); return -1; }
  if(t < GetCurrentTime()) return false;
  //update controller mirror
  int i=GetTrajOffset(t);
  if(i < 0) {
    fprintf(stderr,"Error: inconsistency between milestones and VAL3 server?\n");
    return false;
  }
  else if(i >= numSegments) {
    assert(numSegments + 1 < maxSegments);

    //add a delay
    GetTrajMilestone(curSegment+numSegments+1)=GetTrajMilestone(curSegment+numSegments);
    Vector zero(GetTrajMilestone(curSegment+numSegments).size(),0.0);
    GetTrajTangent(curSegment+i+1) = GetTrajTangent(curSegment+i) = zero;
    GetTrajTime(curSegment+numSegments+1)=t;
    numSegments ++;
  }
  else {
    //add a split
    while(t == GetTrajTime(curSegment+i) && i >= 0) {
      i--;  //split at first segment with t < tsegment
    }
    if(i < 0) {
      fprintf(stderr,"Error: inconsistency between milestones and VAL3 server?\n");
      return false;
    }
    Vector x,v;
    EvalTrajSegment(i,t,x,v);
    GetTrajMilestone(curSegment+i+1) = x;
    GetTrajTangent(curSegment+i+1) = v;
    GetTrajTime(curSegment+i+1) = t;
    numSegments = i+1;
  }
  return true;
}

bool CS8Emulator::ResetTrajectoryRel(double t)
{
  return ResetTrajectoryAbs(GetCurrentTime()+t);
}

void CS8Emulator::AdvanceTime(double t,double tSystem)
{
  assert(t >= currentTime);
  currentTimeUpdateTime = tSystem;
  if(t == currentTime) return;

  //simulate the servo
  //advance q and dq
  double kActualAccelCoeff = 1000.0;
  //double kEstimatedAccelCoeff = 100.0;
  double kEstimatedAccelCoeff = 100.0;
  double kCorrectionCoeff = 1.0;
  double kVelocityGain = 1.0;
  double kPositionGain = 0.3;
  //bool print = GetTrajEndTime() > currentTime;
  bool print = false;
  static vector<double> qLast;
  while(lastServoTime + kServoTimestep <= t) {
    if(print) printf("Servo time %g\n",lastServoTime);
    //evolve current according to hypothesized velocity controller
    for(size_t i=0;i<qCurrent.size();i++) {
      double accel = kActualAccelCoeff*(dqCommand[i]-dqCurrent[i]);
      if(dqCurrent[i]*accel < 0) //decelerating
	accel = Sign(accel)*std::min(fabs(accel),dmax0[i]);
      else
	accel = Sign(accel)*std::min(fabs(accel),amax0[i]);
      dqCurrent[i] += accel*kServoTimestep;
      //internal damping / friction
      dqCurrent[i] *= pow(1.0-kServoDampingRatioPerSecond,kServoTimestep);
      qCurrent[i] += dqCurrent[i]*kServoTimestep;
      if(print) printf(" qcurrent %d %g, dqcurrent %g\n",i,qCurrent[i],dqCurrent[i]);
    }
    
    if(qLast.empty()) qLast = qCurrent;
    //simulate robotMotion loop
    GetTrajConfig(lastServoTime + kServoTimestep,qDesired);
    GetTrajVelocity(lastServoTime + kServoTimestep,dqDesired);
    for(size_t i=0;i<qCurrent.size();i++) {
      if(print) printf(" desired %d %g, dq %g\n",i,qDesired[i],dqDesired[i]);
      qEstimate[i] = qEstimate[i] + kServoTimestep*dqCommand[i];
      if(print) printf(" estimate %g\n",qEstimate[i]);
      qEstimate[i] += kCorrectionCoeff*(qCurrent[i] - qEstimate[i]);
      if(print) printf(" corrected estimate %g\n",qEstimate[i]);
      dqCommand[i] = kPositionGain*(qDesired[i] - qEstimate[i]) / kServoTimestep + kVelocityGain*dqDesired[i];
      if(print) printf(" command %g\n",dqCommand[i]);

      //desired vel = dqDesired
      double dqLast = (qCurrent[i]-qLast[i])/kServoTimestep;
      dqCommand[i] = dqLast + (dqCommand[i]-dqLast)/(kEstimatedAccelCoeff*kServoTimestep);
    }
    qLast = qCurrent;
    
    //what happens on setVelCmd?
    //nothing...?

    lastServoTime += kServoTimestep;
    if(print) getchar();
  }
  currentTime = t;

  //advance
  int ns=GetTrajOffset(t);
  numSegments -= ns;
  curSegment += ns;
  if(numSegments == 0) {  //hit end of queue
    if(ns > 0 && kCheckNonzeroVelocity) { //help debugging
      numSegments++;
      curSegment--;
      Vector x,v;
      EvalTrajSegment(0,GetTrajTime(curSegment+1),x,v);
      numSegments--;
      curSegment++;
      for(size_t i=0;i<v.size();i++) {
	if(fabs(v[i]) > dmax[i]/maxRate) {
	  fprintf(stderr,"CS8Emulator: Hit end of motion queue with nonzero velocity %g!\n",v[i]);
	  motionError = true;
	  break;
	}
      }
    }
    GetTrajTime(curSegment) = t;
  }
}

int CS8Emulator::AddMilestone(double t,const Vector& q,const Vector& v)
{
  if(motionError) { printf("CS8Emulator: attempted to add milestone after motion error\n"); return -1; }
  if(numSegments + 1 >= maxSegments) { printf("CS8Emulator: attempted to add milestone at max segments\n"); return -1; }
  if(t < GetTrajEndTime())  { printf("CS8Emulator: attempted to add milestone before end time\n"); return -1; }
  //append
  if(numSegments == 0) {
    //initialize time of start of current segment 
    GetTrajTime(curSegment) = GetCurrentTime();
    //initialize velocity of current segment
    Vector& v0=GetTrajTangent(curSegment);
    v0.resize(q.size());
    fill(v0.begin(),v0.end(),0.0);
  }
  assert(q.size()==v.size());

  GetTrajMilestone(curSegment+numSegments+1) = q;
  GetTrajTangent(curSegment+numSegments+1) = v;
  GetTrajTime(curSegment+numSegments+1) = t;
  numSegments ++;
  return numSegments;
}


bool CS8Emulator::AppendMilestonesQuiet(const std::vector<double>& ts,const std::vector<Vector>& q,const std::vector<Vector>& v)
{
  if(numSegments + (int)ts.size() > maxSegments) return false;
  if(motionError) { printf("CS8Emulator: attempted to append milestones after motion error\n"); return -1; }
  if(ts[0] < GetTrajEndTime()) {
    if(GetTrajEndTime() == GetCurrentTime())
      fprintf(stderr,"Trying to append milestones before current time, %g<%g\n",ts[0],GetCurrentTime());
    else
      fprintf(stderr,"Trying to append milestones before end of motion queue, %g<%g\n",ts[0],GetTrajEndTime());
    return false;
  }
  for(size_t i=1;i<ts.size();i++)
    assert(ts[i] >= ts[i-1]); 
  //append
  if(numSegments == 0) {
    //initialize time of start of current segment 
    GetTrajTime(curSegment) = GetCurrentTime();
    //initialize velocity of current segment
    Vector& v0=GetTrajTangent(curSegment);
    v0.resize(GetTrajMilestone(curSegment+numSegments).size());
    fill(v0.begin(),v0.end(),0.0);
  }
  for(size_t i=0;i<ts.size();i++) {
    assert(q[i].size()==GetTrajMilestone(curSegment+numSegments).size());
    assert(q[i].size()==v[i].size());
    //assert(ts[i]>0);
    GetTrajMilestone(curSegment+numSegments+1) = q[i];
    GetTrajTangent(curSegment+numSegments+1) = v[i];
    GetTrajTime(curSegment+numSegments+1) = ts[i];
    numSegments ++;
  }
  return true;
}
