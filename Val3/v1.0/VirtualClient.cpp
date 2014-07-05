#include "VirtualClient.h"
#include <tinyxml/tinyxml.h>
#include <assert.h>
#include <math.h>
#include <algorithm>
using namespace std;

VirtualTrajClient::VirtualTrajClient()
  :curSegment(0),numSegments(0)
{}

void VirtualTrajClient::Disconnect()
{
  milestones.clear();
  times.clear();
  curSegment = 0;
  numSegments = 0;
}

bool VirtualTrajClient::Sync(TrajClient& t)
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
  
  maxRate = t.Rate();

  milestones.resize(maxSegments+1);
  times.resize(maxSegments+1);
  curSegment = 0;
  numSegments = 0;
  //get current status
  res = t.GetEndConfig(milestones[0]);
  assert(res);
  times[0] = t.GetTrajEndTime();
  return true;
}

bool VirtualTrajClient::SetupVirtual(const char* xmlFile)
{
  virtualTimer.Reset();

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
  return true;
}





bool VirtualTrajClient::GetConfig(Vector& q)
{
  double t=virtualTimer.ElapsedTime();
  GetTrajConfig(t,q);
  return true;
}

bool VirtualTrajClient::GetVelocity(Vector& v)
{
  double t=virtualTimer.ElapsedTime();
  GetTrajVelocity(t,v);
  return true;
}

bool VirtualTrajClient::GetTransform(Vector& x)
{
  double t=virtualTimer.ElapsedTime();
  Vector q;
  GetTrajConfig(t,q);
  fprintf(stderr,"Virtual GetTransform not implemented yet\n");
  abort();
  return true;
}

bool VirtualTrajClient::GetJointLimits(Vector& jmin,Vector& jmax)
{
  jmin = qmin;
  jmax = qmax;
  return true;
}

bool VirtualTrajClient::GetVelocityLimits(Vector& vmax)
{
  vmax = this->vmax;
  return true;
}

bool VirtualTrajClient::GetAccelerationLimits(Vector& amax)
{
  amax = this->amax;
  return true;
}

bool VirtualTrajClient::GetDecelerationLimits(Vector& dmax)
{
  dmax = this->dmax;
  return true;
}

int VirtualTrajClient::GetMaxSegments()
{
  return this->maxSegments;
}

int VirtualTrajClient::GetCurSegments()
{
  AdvanceTime(virtualTimer.ElapsedTime());
  return numSegments;
}

int VirtualTrajClient::GetRemainingSegments()
{
  return GetMaxSegments()-GetCurSegments();
}

double VirtualTrajClient::GetCurrentTime()
{
  return virtualTimer.ElapsedTime();
}

double VirtualTrajClient::GetTrajEndTime()
{
  return GetTrajTime(curSegment+numSegments);
}


double VirtualTrajClient::GetTrajDuration()
{
  return GetTrajEndTime()-GetCurrentTime();
}

bool VirtualTrajClient::GetEndConfig(Vector& jend) const
{
  jend = GetTrajMilestone(curSegment+numSegments);
  return true;
}

bool VirtualTrajClient::GetEndVelocity(Vector& vend)
{
  if(numSegments == 0 || GetCurrentTime() > GetTrajTime(curSegment+numSegments)) {
    vend.resize(GetTrajMilestone(curSegment).size());
    fill(vend.begin(),vend.end(),0.0);
  }
  else {
    vend=GetTrajMilestone(curSegment+numSegments);
    const Vector& v0=GetTrajMilestone(curSegment+numSegments-1);
    double dt = GetTrajTime(curSegment+numSegments)-GetTrajTime(curSegment+numSegments-1);
    for(size_t i=0;i<vend.size();i++)
      vend[i] = (vend[i]-v0[i])/dt;
  }
  return true;
}

bool VirtualTrajClient::CheckTrajectory()
{
  //TODO: actually check it
  return true;
}

bool VirtualTrajClient::SetJointLimits(const Vector& qmin,const Vector& qmax)
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

bool VirtualTrajClient::SetVelocityLimits(const Vector& vmax)
{
  assert(vmax.size()==this->vmax.size());
  for(size_t i=0;i<vmax.size();i++) {
    assert(vmax[i] >= 0);
    this->vmax[i] = std::min(this->vmax0[i],vmax[i]);
  }
  return true;
}

bool VirtualTrajClient::SetAccelerationLimits(const Vector& amax)
{
  assert(amax.size()==this->amax.size());
  for(size_t i=0;i<amax.size();i++) {
    assert(amax[i] >= 0);
    this->amax[i] = std::min(this->amax0[i],amax[i]);
  }
  return true;
}

bool VirtualTrajClient::SetDecelerationLimits(const Vector& dmax)
{
  assert(amax.size()==this->amax.size());
  for(size_t i=0;i<amax.size();i++) {
    assert(dmax[i] >= 0);
    this->dmax[i] = std::min(this->dmax0[i],dmax[i]);
  }
  return true;
}

int VirtualTrajClient::GetTrajSegment(double t) const
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

int VirtualTrajClient::GetTrajOffset(double t) const
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


void VirtualTrajClient::GetTrajConfig(double t,Vector& x) const
{
  int i=GetTrajOffset(t);
  if(i < 0) { x=GetTrajMilestone(curSegment); return; }
  else if(i >= numSegments) { x=GetTrajMilestone(curSegment+numSegments); return; }
  else {
    i += curSegment;
    double u=(t-GetTrajTime(i))/(GetTrajTime(i+1)-GetTrajTime(i));
    const Vector& m0 = GetTrajMilestone(i);
    const Vector& m1 = GetTrajMilestone(i+1);
    x = m0;
    for(size_t j=0;j<m0.size();j++)
      x[j] += u*(m1[j]-m0[j]);
  }
}

void VirtualTrajClient::GetTrajVelocity(double t,Vector& x) const
{
  int i=GetTrajOffset(t);
  if(i < 0) { 
    x.resize(GetTrajMilestone(curSegment).size());
    fill(x.begin(),x.end(),0.0);
  }
  else if(i >= numSegments) { 
    x.resize(GetTrajMilestone(curSegment).size());
    fill(x.begin(),x.end(),0.0);
  }
  else {
    i+=curSegment;
    double dt=(GetTrajTime(i+1)-GetTrajTime(i));
    const Vector& m0 = GetTrajMilestone(i);
    const Vector& m1 = GetTrajMilestone(i+1);
    x = m0;
    for(size_t j=0;j<m0.size();j++)
      x[j] = (m1[j]-m0[j])/dt;
  }
}

bool VirtualTrajClient::ResetTrajectoryAbs(double t)
{
  if(t < GetCurrentTime()) return false;
  //update controller mirror
  int i=GetTrajOffset(t);
  if(i < 0) {
    fprintf(stderr,"Error: inconsistency between milestones and VAL3 server?\n");
    return false;
  }
  else if(i >= numSegments) {
    //add a delay
    GetTrajMilestone(curSegment+numSegments+1)=GetTrajMilestone(curSegment+numSegments);
    GetTrajTime(curSegment+numSegments+1)=t;
    numSegments++;
  }
  else {
    //add a split
    while(t == GetTrajTime(curSegment+i) && i >= 0) {
      i--;
    }
    if(i < 0) {
      fprintf(stderr,"Error: inconsistency between milestones and VAL3 server?\n");
      return false;
    }
    numSegments = i+1;
    double u = (t-GetTrajTime(curSegment+i))/(GetTrajTime(curSegment+i+1)-GetTrajTime(curSegment+i));
    GetTrajTime(curSegment+i+1) = t;
    const Vector& m0=GetTrajMilestone(curSegment+i);
    const Vector& m1=GetTrajMilestone(curSegment+i+1);
    Vector& x=GetTrajMilestone(curSegment+i+1);
    for(size_t j=0;j<m1.size();j++)
      x[j] = m0[j] + u*(m1[j]-m0[j]);
  }
  return true;
}

bool VirtualTrajClient::ResetTrajectoryRel(double t)
{
  return ResetTrajectoryAbs(GetCurrentTime()+t);
}

void VirtualTrajClient::AdvanceTime(double t)
{
  //advance
  int ns=GetTrajOffset(t);
  numSegments -= ns;
  curSegment += ns;
  if(numSegments == 0)
    GetTrajTime(curSegment) = t;
}

bool VirtualTrajClient::AppendMilestonesQuiet(const std::vector<double>& dts,const std::vector<Vector>& q)
{
  assert(numSegments + (int)dts.size() < maxSegments);
  //append
  if(numSegments == 0) {
    //initialize time of start of current segment 
    GetTrajTime(curSegment) = GetCurrentTime();
  }
  for(size_t i=0;i<dts.size();i++) {
    assert(dts[i]>0);
    GetTrajMilestone(curSegment+numSegments+1) = q[i];
    GetTrajTime(curSegment+numSegments+1) = GetTrajTime(curSegment+numSegments)+dts[i];
    numSegments ++;
  }
  return true;
}
