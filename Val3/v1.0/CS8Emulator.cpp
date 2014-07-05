#include "CS8Emulator.h"
#include <tinyxml/tinyxml.h>
#include <assert.h>
#include <math.h>
#include <algorithm>
using namespace std;

const static bool kCheckNonzeroVelocity = true;

#define QUADRATIC_INTERPOLATE 1

//quadratic interpolation scheme defined in TrajServer
void QInterpolate(double tp,const CS8Emulator::Vector& qp,
		 double t1,const CS8Emulator::Vector& q1,
		 double t2,const CS8Emulator::Vector& q2,
		 double t,CS8Emulator::Vector& qt,CS8Emulator::Vector& dqt)
{
#if QUADRATIC_INTERPOLATE
  assert(t2 >= t && t >= t1);
  assert(t2 >= tp);
  assert(qp.size()==q1.size());
  assert(q2.size()==q1.size());
  qt.resize(q1.size());
  dqt.resize(q1.size());
  double dt2 = t2-t1;
  double dt0 = tp-t1;
  double denom1 = dt0*(dt0-dt2);
  double denom2 = dt2*(dt0-dt2);

  if(std::min(denom1,-denom2) < 0.000032) {
    //do a linear interpolation
    assert(dt2 > 0);
    double u = (t-t1)/dt2;
    for(size_t i=0;i<q1.size();i++) {
      qt[i] = (1.0-u)*q1[i] + u*q2[i];
      dqt[i] = (q2[i]-q1[i])/dt2;
    }
  }
  else {
    double u = t-t1;
    //evaluate derivative
    double coef0 = (2*u-dt2)/denom1;
    double coef2 = (-2*u+dt0)/denom1;
    for(size_t i=0;i<q1.size();i++) 
      dqt[i] = coef0*(qp[i]-q1[i]) + coef2*(qp[i]-q1[i]);
    //evaluate position
    coef0 = (u*u-u*dt2)/denom1;
    coef2 = (-u*u+u*dt0)/denom2;
    for(size_t i=0;i<q1.size();i++) 
      qt[i] = (1.0-coef0-coef2)*q1[i] + coef0*qp[i] + coef2*q2[i];
  }
#else
  double u = (t-t1)/dt2;
  for(size_t i=0;i<q1.size();i++) {
    qt[i] = (1.0-u)*q1[i] + u*q2[i];
    dqt[i] = (q2[i]-q1[i])/dt2;
  }
#endif // QUADRATIC_INTERPOLATE
}



CS8Emulator::CS8Emulator()
  :curSegment(0),numSegments(0)
{}

void CS8Emulator::Disconnect()
{
  milestones.clear();
  times.clear();
  curSegment = 0;
  numSegments = 0;
  currentTime = 0;
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

  milestones.resize(maxSegments+1);
  times.resize(maxSegments+1);
  curSegment = 0;
  numSegments = 0;
  while(t.GetTrajEndTime() > t.GetCurrentTime()) {
    printf("Waiting %gs until the motion stops...\n",t.GetTrajEndTime()-t.GetCurrentTime());
    usleep(int(2.0*(t.GetTrajEndTime()-t.GetCurrentTime())*1000000));
  }
  //get current status
  res = t.GetEndConfig(milestones[0]);
  assert(res);
  times[0] = t.GetTrajEndTime();
  currentTime = t.GetCurrentTime();
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
  times.resize(maxSegments+1);
  curSegment = 0;
  numSegments = 0;
  //get current status
  milestones[0]=q;
  times[0] = 0;
  currentTime = 0;
  return true;
}





bool CS8Emulator::GetConfig(Vector& q)
{
  double t=currentTime;
  GetTrajConfig(t,q);
  return true;
}

bool CS8Emulator::GetVelocity(Vector& v)
{
  double t=currentTime;
  GetTrajVelocity(t,v);
  return true;
}

bool CS8Emulator::GetTransform(Vector& x)
{
  double t=currentTime;
  Vector q;
  GetTrajConfig(t,q);
  fprintf(stderr,"Virtual GetTransform not implemented yet\n");
  abort();
  return true;
}

bool CS8Emulator::GetJointLimits(Vector& jmin,Vector& jmax)
{
  jmin = qmin;
  jmax = qmax;
  return true;
}

bool CS8Emulator::GetVelocityLimits(Vector& vmax)
{
  vmax = this->vmax;
  return true;
}

bool CS8Emulator::GetAccelerationLimits(Vector& amax)
{
  amax = this->amax;
  return true;
}

bool CS8Emulator::GetDecelerationLimits(Vector& dmax)
{
  dmax = this->dmax;
  return true;
}

int CS8Emulator::GetMaxSegments()
{
  return this->maxSegments;
}

int CS8Emulator::GetCurSegments()
{
  return numSegments;
}

int CS8Emulator::GetRemainingSegments()
{
  return GetMaxSegments()-GetCurSegments();
}

double CS8Emulator::GetCurrentTime()
{
  return currentTime;
}

double CS8Emulator::GetTrajEndTime()
{
  return GetTrajTime(curSegment+numSegments);
}


double CS8Emulator::GetTrajDuration()
{
  return GetTrajEndTime()-GetCurrentTime();
}

bool CS8Emulator::GetEndConfig(Vector& jend) const
{
  jend = GetTrajMilestone(curSegment+numSegments);
  return true;
}

bool CS8Emulator::GetEndVelocity(Vector& vend)
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
    bool duplicateStart = (curSegment == 0);
    if(numSegments == maxSegments && i==0) { 
      fprintf(stderr,"Warning, motion queue is filled, so using standard quadratic interpolant would wrap over\n");
      duplicateStart = true;
    }
    i += curSegment;
    const Vector& m1 = GetTrajMilestone(i);
    const Vector& m2 = GetTrajMilestone(i+1);
    const Vector& mp = (!duplicateStart ? GetTrajMilestone(i-1) : m1);
    double t1=GetTrajTime(i);
    double t2=GetTrajTime(i+1);
    double tp = (!duplicateStart ? GetTrajTime(i-1) : t1-(t2-t1));
    QInterpolate(tp,mp,t1,m1,t2,m2,t,x,v);
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
  if(t < GetCurrentTime()) return false;
  //update controller mirror
  int i=GetTrajOffset(t);
  if(i < 0) {
    fprintf(stderr,"Error: inconsistency between milestones and VAL3 server?\n");
    return false;
  }
  else if(i >= numSegments) {
    assert(numSegments + 1 < maxSegments);

#if QUADRATIC_INTERPOLATE
    if(GetTrajTime(curSegment+numSegments) < t-1.0/maxRate) {
      assert(numSegments + 2 < maxSegments);
      GetTrajTime(curSegment+numSegments+1)=t-1.0/maxRate;
      //add an extra milestone in the middle
      GetTrajMilestone(curSegment+numSegments+1)=GetTrajMilestone(curSegment+numSegments);
      numSegments ++;
    }
#endif
    //add a delay
    GetTrajMilestone(curSegment+numSegments+1)=GetTrajMilestone(curSegment+numSegments);
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
	  break;
	}
      }
    }
    GetTrajTime(curSegment) = t;
  }
}

int CS8Emulator::AddMilestone(double dt,const Vector& q)
{
  if(numSegments + 1 >= maxSegments) return false;
  //append
  if(numSegments == 0) {
    //initialize time of start of current segment 
    GetTrajTime(curSegment) = GetCurrentTime();
    //for quadratic interpolant: initialize time of prior segment
#if QUADRATIC_INTERPOLATE
    if(curSegment>0) {
      GetTrajTime(curSegment-1) = GetTrajTime(curSegment)-dt;
      GetTrajMilestone(curSegment-1) = GetTrajMilestone(curSegment);
    }
#endif
  }
  GetTrajMilestone(curSegment+numSegments+1) = q;
  GetTrajTime(curSegment+numSegments+1) = GetTrajTime(curSegment+numSegments)+dt;
  numSegments ++;
  return true;
}


bool CS8Emulator::AppendMilestonesQuiet(const std::vector<double>& dts,const std::vector<Vector>& q)
{
  if(numSegments + (int)dts.size() > maxSegments) return false;
  //append
  if(numSegments == 0) {
    //initialize time of start of current segment 
    GetTrajTime(curSegment) = GetCurrentTime();
#if QUADRATIC_INTERPOLATE
    if(curSegment>0 && dts.size()>0) {
      GetTrajTime(curSegment-1) = GetTrajTime(curSegment)-dts[0];
      GetTrajMilestone(curSegment-1) = GetTrajMilestone(curSegment);
    }
#endif
  }
  for(size_t i=0;i<dts.size();i++) {
    assert(dts[i]>0);
    GetTrajMilestone(curSegment+numSegments+1) = q[i];
    GetTrajTime(curSegment+numSegments+1) = GetTrajTime(curSegment+numSegments)+dts[i];
    numSegments ++;
  }
  return true;
}
