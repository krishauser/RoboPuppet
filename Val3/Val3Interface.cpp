#include "Val3Interface.h"
#include "Modeling/Conversions.h"
#include <spline/PiecewisePolynomial.h>
#include <tinyxml.h>
#include <utils/stringutils.h>
using namespace Spline;

template <class T>
void ReadVector(const string& s,vector<T>& vec)
{
  vec.resize(0);
  stringstream ss(s);
  T temp;
  while(ss >> temp) 
    vec.push_back(temp);
}

void ToTrajClient(const ParabolicRamp::ParabolicRampND& ramp,ParabolicRamp::ParabolicRampND& castRamp,const KlamptToVal3Interface* iface)
{
  assert(&ramp != &castRamp);
  assert(ramp.IsValid());
  castRamp.endTime = ramp.endTime;
  iface->ConfigToTrajClient(ramp.x0,castRamp.x0);
  iface->VectorToTrajClient(ramp.dx0,castRamp.dx0);
  iface->ConfigToTrajClient(ramp.x1,castRamp.x1);
  iface->VectorToTrajClient(ramp.dx1,castRamp.dx1);

  castRamp.ramps.resize(iface->robotFromModel.size());
  for(size_t i=0;i<castRamp.ramps.size();i++) {
    if(iface->robotFromModel[i].index < 0) {
      castRamp.ramps[i].SetConstant(iface->robotFromModel[i].offset);
    }
    else {
      int k=iface->robotFromModel[i].index;
      double scale=iface->robotFromModel[i].scale;
      double offset=iface->robotFromModel[i].offset;
      castRamp.ramps[i].x0 = ramp.ramps[k].x0*scale+offset;
      castRamp.ramps[i].x1 = ramp.ramps[k].x1*scale+offset;
      castRamp.ramps[i].dx0 = ramp.ramps[k].dx0*scale;
      castRamp.ramps[i].dx1 = ramp.ramps[k].dx1*scale;
      assert(FuzzyEquals(castRamp.ramps[i].x0,castRamp.x0[i]));
      assert(FuzzyEquals(castRamp.ramps[i].x1,castRamp.x1[i]));
      assert(FuzzyEquals(castRamp.ramps[i].dx0,castRamp.dx0[i]));
      assert(FuzzyEquals(castRamp.ramps[i].dx1,castRamp.dx1[i]));
      castRamp.ramps[i].tswitch1 = ramp.ramps[k].tswitch1;
      castRamp.ramps[i].tswitch2 = ramp.ramps[k].tswitch2;
      castRamp.ramps[i].ttotal = ramp.ramps[k].ttotal;
      castRamp.ramps[i].a1 = ramp.ramps[k].a1*scale;
      castRamp.ramps[i].v = ramp.ramps[k].v*scale;
      castRamp.ramps[i].a2 = ramp.ramps[k].a2*scale;
    }
  }
  //sometimes this will crap about because of numerical errors
  //assert(castRamp.IsValid());
}


KlamptToVal3Interface::KlamptToVal3Interface()
{}

KlamptToVal3Interface::~KlamptToVal3Interface()
{
}

bool KlamptToVal3Interface::LoadSetup(const char* setupXml)
{
  Disconnect();
  TiXmlDocument doc;
  if(!doc.LoadFile(setupXml)) {
    fprintf(stderr,"Error reading XML file %s\n",setupXml);
    return false;
  }
  if(0!=strcmp(doc.RootElement()->Value(),"val3setup")) {
    fprintf(stderr,"Wrong type of root element %s\n",doc.RootElement()->Value());
    return false;
  }
  TiXmlElement* e;
  e=doc.RootElement()->FirstChildElement("model");
  if(!e) {
    fprintf(stderr,"Unable to read model element\n");
    return false;
  }
  if(e->QueryValueAttribute("name",&modelName) != TIXML_SUCCESS) {
    modelName = "";
  }
  if(e->QueryValueAttribute("file",&modelFile) != TIXML_SUCCESS) {
    modelFile = "";
  }

  e=doc.RootElement()->FirstChildElement("motion");
  if(e) {
    if(e->Attribute("rate")) {
      rate = e->Attribute("rate");
    }
    if(e->Attribute("qmin")) {
      qmin = e->Attribute("qmin");
    }
    if(e->Attribute("qmax")) {
      qmax = e->Attribute("qmax");
    }
    if(e->Attribute("vmax")) {
      vmax = e->Attribute("vmax");
    }
    if(e->Attribute("amax")) {
      amax = e->Attribute("amax");
    }
    if(e->Attribute("dmax")) {
      dmax = e->Attribute("dmax");
    }
    else
      dmax = amax;
  }

  IndexMapping def;
  def.index = -1;
  def.scale = 0.0;
  def.offset = 0.0;
  modelFromRobot.resize(0);
  robotFromModel.resize(0);
  e = doc.RootElement()->FirstChildElement("map");
  while(e) {
    int irobot=-1,imodel=-1;
    if(e->QueryValueAttribute("robot",&irobot) != TIXML_SUCCESS) {
      irobot = -1;
    }
    if(e->QueryValueAttribute("model",&imodel) != TIXML_SUCCESS) {
      imodel = -1;
    }
    double scale=1.0,offset=0.0,value=0.0;
    if(e->QueryValueAttribute("offset",&offset) != TIXML_SUCCESS) {
      offset = 0.0;
    }
    if(e->QueryValueAttribute("scale",&scale) != TIXML_SUCCESS) {
      scale = 1.0;
    }
    if(e->QueryValueAttribute("value",&value) == TIXML_SUCCESS) {
      offset = value;
      scale = 0.0;
      if(irobot >= 0 && imodel >= 0) {
	fprintf(stderr,"Can't have a fixed value mapping for a robot->model\n");
	return false;
      }
    }
    IndexMapping mapping;
    mapping.index = irobot;
    mapping.scale = scale;
    mapping.offset = offset;
    if(imodel >= 0) {
      modelFromRobot.resize(imodel+1,def);
      if(modelFromRobot[imodel].index > 0) {
	fprintf(stderr,"Multiple robot joint mappings on %d\n",imodel);
	return false;
      }
      modelFromRobot[imodel] = mapping;
    }
    mapping.index = imodel;
    if(scale == 0.0) {
      mapping.scale = 0.0;
      mapping.offset = offset;
    }
    else {
      mapping.scale = 1.0/scale;
      mapping.offset = -offset/scale;
    }
    if(irobot >= 0) {
      robotFromModel.resize(irobot+1,def);
      if(robotFromModel[irobot].index > 0) {
	fprintf(stderr,"Multiple model joint mappings on %d\n",irobot);
	return false;
      }
      robotFromModel[irobot] = mapping;
    }
    e=e->NextSiblingElement("map");
  }

  printf("Robot to model mapping\n");
  for(size_t i=0;i<robotFromModel.size();i++)
    printf("%d: %d %g %g\n",i,robotFromModel[i].index,robotFromModel[i].scale,robotFromModel[i].offset);

  if(robotFromModel.size() != 6) {
    fprintf(stderr,"Invalid size of robot mapping\n");
    return false;
  }
  return true;
}

bool KlamptToVal3Interface::Connect(const char* host,int port,double version)
{
  if(version != 1.0) { 
    fprintf(stderr,"KlamptToVal3Interface: only version 1.0 is currently supported\n");
    return false;
  }
  Disconnect();
  if(!client.Connect(host,port)) return false;

  //perform some setup
  if(!rate.empty()) {
    double val;
    stringstream ss;
    double scale = 1.0;
    if(rate[rate.length()-1]=='%') {
      //read off a percentage
      ss.str(rate.substr(0,rate.length()-1));
      scale = 0.01;
    }
    else
      ss.str(rate);
    ss >> val;
    val *= scale;
    if(!ss || val < 0 || val > 1.0) {
      fprintf(stderr,"Warning: ignoring rate spec: %s\n",rate.c_str());
      fprintf(stderr,"  Press enter to continue\n");
      getchar();
    }
    client.SetSpeedScale(val);
  }

  if(!qmin.empty() && !qmax.empty()) {
    vector<double> vqmin,vqmax;
    ReadVector(qmin,vqmin);
    ReadVector(qmax,vqmax);

    //Delay until a valid initial configuration is found
    while(true) {
      vector<double> q;
      client.GetConfig(q);
      bool feas=true;
      for(size_t i=0;i<q.size();i++) {
	if(q[i] < vqmin[i] || q[i] > vqmax[i]) {
	  feas=false;
	  break;
	}
      }
      if(feas) break;
      else {
	printf("\n");
	printf("Robot's current config exceeds desired joint limits!\n");
	printf("Please move them into the desired joint limits manually\n");
	printf("and then press Enter to continue...\n");
	for(size_t i=0;i<q.size();i++) {
	  if(q[i] < vqmin[i])
	    printf("Joint %d value %g < limit %g\n",i,q[i],vqmin[i]);
	  else if(q[i] > vqmax[i]) 
	    printf("Joint %d value %g > limit %g\n",i,q[i],vqmax[i]);
	}
	getchar();
      }
    }
    client.SetJointLimits(vqmin,vqmax);
  }
  if(!vmax.empty()) {
    vector<double> vvmax;
    ReadVector(vmax,vvmax);
    client.SetVelocityLimits(vvmax);
  }
  if(!amax.empty()) {
    vector<double> vamax;
    ReadVector(amax,vamax);
    client.SetAccelerationLimits(vamax);
    if(dmax.empty())
      client.SetDecelerationLimits(vamax);
  }
  if(!dmax.empty()) {
    vector<double> vdmax;
    ReadVector(dmax,vdmax);
    client.SetDecelerationLimits(vdmax);
    if(amax.empty())
      client.SetAccelerationLimits(vdmax);
  }
  nextSendTime = 0;
  client.SyncTimers(systemTimer.ElapsedTime());
  return true;
}

bool KlamptToVal3Interface::ConnectInternalEmulator(const char* specFile)
{
  Disconnect();
  if(!client.SetVirtual(specFile)) return false;

  //perform some setup
  if(!rate.empty()) {
    double val;
    stringstream ss;
    double scale = 1.0;
    if(rate[rate.length()-1]=='%') {
      //read off a percentage
      ss.str(rate.substr(0,rate.length()-1));
      scale = 0.01;
    }
    else
      ss.str(rate);
    ss >> val;
    val *= scale;
    if(!ss || val < 0 || val > 1.0) {
      fprintf(stderr,"Warning: ignoring rate spec: %s\n",rate.c_str());
      fprintf(stderr,"  Press enter to continue\n");
      getchar();
    }
    client.SetSpeedScale(val);
  }

  if(!qmin.empty() && !qmax.empty()) {
    vector<double> vqmin,vqmax;
    ReadVector(qmin,vqmin);
    ReadVector(qmax,vqmax);

    //Set a valid initial configuration is found
    vector<double> q;
    client.GetConfig(q);
    for(size_t i=0;i<q.size();i++) {
      if(q[i] < vqmin[i])
	q[i] = vqmin[i];
      if(q[i] > vqmax[i]) 
	q[i] = vqmax[i];
    }
    //set the initial configuration
    client.MoveTo(q);
    client.SetJointLimits(vqmin,vqmax);
  }
  if(!vmax.empty()) {
    vector<double> vvmax;
    ReadVector(vmax,vvmax);
    client.SetVelocityLimits(vvmax);
  }
  if(!amax.empty()) {
    vector<double> vamax;
    ReadVector(amax,vamax);
    client.SetAccelerationLimits(vamax);
    if(dmax.empty())
      client.SetDecelerationLimits(vamax);
  }
  if(!dmax.empty()) {
    vector<double> vdmax;
    ReadVector(dmax,vdmax);
    client.SetDecelerationLimits(vdmax);
    if(amax.empty())
      client.SetAccelerationLimits(vdmax);
  }
  nextSendTime = 0;
  return true;
}

void KlamptToVal3Interface::Disconnect()
{
  client.Disconnect();
}

void KlamptToVal3Interface::SendPoll()
{
  double t = systemTimer.ElapsedTime();
  if(t >= nextSendTime) {
    client.AdvanceMirrorTime(client.EstimateCurrentTime(t),t);
    double sleepTime = client.SendPoll(t);
    t = systemTimer.ElapsedTime();
    nextSendTime = t + sleepTime;
    client.AdvanceMirrorTime(client.EstimateCurrentTime(t),t);
  }
}

void KlamptToVal3Interface::ConfigToTrajClient(const Config& q,TrajClient::Vector& v) const
{
  v.resize(robotFromModel.size());
  for(size_t i=0;i<robotFromModel.size();i++) {
    double scale=robotFromModel[i].scale;
    double offset=robotFromModel[i].offset;
    if(robotFromModel[i].index >= 0) {
      Assert(robotFromModel[i].index < q.n);
      v[i] = scale*q(robotFromModel[i].index)+offset;
    }
    else
      v[i] = offset;
  }
}

void KlamptToVal3Interface::ConfigFromTrajClient(const TrajClient::Vector& v,Config& q) const
{
  q.resize(modelFromRobot.size());
  for(size_t i=0;i<modelFromRobot.size();i++) {
    double scale=modelFromRobot[i].scale;
    double offset=modelFromRobot[i].offset;
    if(modelFromRobot[i].index >= 0) {
      Assert(modelFromRobot[i].index < (int)v.size());
      q[i] = scale*v[modelFromRobot[i].index]+offset;
    }
    else
      q[i] = offset;
  }
}

void KlamptToVal3Interface::VectorToTrajClient(const Vector& q,TrajClient::Vector& v) const
{
  v.resize(robotFromModel.size());
  for(size_t i=0;i<robotFromModel.size();i++) {
    double scale=robotFromModel[i].scale;
    if(robotFromModel[i].index >= 0) {
      Assert(robotFromModel[i].index < q.n);
      v[i] = scale*q(robotFromModel[i].index);
    }
    else
      v[i] = 0.0;
  }
}


void KlamptToVal3Interface::VectorFromTrajClient(const TrajClient::Vector& v,Vector& q) const
{
  q.resize(modelFromRobot.size());
  for(size_t i=0;i<modelFromRobot.size();i++) {
    double scale=modelFromRobot[i].scale;
    if(modelFromRobot[i].index >= 0) {
      Assert(modelFromRobot[i].index < (int)v.size());
      q[i] = scale*v[modelFromRobot[i].index];
    }
    else
      q[i] = 0.0;
  }
}


void KlamptToVal3Interface::UpdateModelCurrent(Robot* model)
{
  GetConfig(model->q);
  GetVelocity(model->dq);
  GetJointLimits(model->qMin,model->qMax);
  GetVelocityLimits(model->velMax);
  GetAccelerationLimits(model->accMax);
  //there may be some minor numerical errors...
  for(int i=0;i<model->qMin.n;i++) {
    if(model->qMin(i) < model->qMax(i)) {
      model->qMin(i) += 1e-3;
      model->qMax(i) -= 1e-3;
    }
  }
  //velocity and acceleration limits are checked by finite differencing...
  //for(int i=0;i<model->velMax.n;i++) model->velMax(i) = Max(model->velMax(i)-Epsilon,0.0);
  //for(int i=0;i<model->velMax.n;i++) model->accMax(i) = Max(model->accMax(i)-Epsilon,0.0);
  for(int i=0;i<model->velMax.n;i++) model->velMax(i) = model->velMax(i)*0.8;
  for(int i=0;i<model->velMax.n;i++) model->accMax(i) = model->accMax(i)*0.8;
  model->UpdateFrames();
  for(size_t i=0;i<model->drivers.size();i++) {
    if(model->drivers[i].type == RobotJointDriver::Normal || model->drivers[i].type == RobotJointDriver::Translation || model->drivers[i].type == RobotJointDriver::Rotation) {
      int itemp = model->drivers[i].linkIndices[0];
      model->drivers[i].qmin = model->qMin(itemp);
      model->drivers[i].qmax = model->qMax(itemp);
      model->drivers[i].vmin = -model->velMax(itemp);
      model->drivers[i].vmax = model->velMax(itemp);
      model->drivers[i].amin = -model->accMax(itemp);
      model->drivers[i].amax = model->accMax(itemp);
    }
  }
}

///appends a move to a new configuration
SafeTrajClient::MotionResult KlamptToVal3Interface::MoveTo(const Config& q,double rate,double dt)
{
  TrajClient::Vector v;
  ConfigToTrajClient(q,v);
  return client.MoveTo(v,rate,dt);
}

SafeTrajClient::MotionResult KlamptToVal3Interface::MoveToImmediate(double dt,const Config& q,double rate)
{
  TrajClient::Vector v;
  ConfigToTrajClient(q,v);
  return client.MoveToImmediate(systemTimer.ElapsedTime(),dt,v,rate);
}

SafeTrajClient::MotionResult KlamptToVal3Interface::MoveRamp(const ParabolicRamp::ParabolicRampND& ramp)
{
  AdvanceTime();
  return MoveRampImmediate(-1,ramp);
}

SafeTrajClient::MotionResult KlamptToVal3Interface::MoveRampImmediate(double tstart,const ParabolicRamp::ParabolicRampND& ramp)
{
  ParabolicRamp::ParabolicRampND castRamp;
  ToTrajClient(ramp,castRamp,this);
  return client.AddTrajectoryImmediate(tstart,Cast(castRamp));
}

SafeTrajClient::MotionResult KlamptToVal3Interface::MoveRamps(const std::vector<ParabolicRamp::ParabolicRampND>& ramps)
{
  AdvanceTime();
  return MoveRampsImmediate(-1,ramps);
}

SafeTrajClient::MotionResult KlamptToVal3Interface::MoveRampsImmediate(double tstart,const std::vector<ParabolicRamp::ParabolicRampND>& ramps)
{
  if(ramps.empty()) {
    if(tstart >= 0) {
      //TODO: check if this reset is ok
      return client.ResetTrajectoryAbs(tstart);
    }
    return SafeTrajClient::Success;
  }
  for(size_t i=1;i<ramps.size();i++) {
    assert(Vector(ramps[i].x0).isEqual(Vector(ramps[i-1].x1),1e-7));
    assert(Vector(ramps[i].dx0).isEqual(Vector(ramps[i-1].dx1),1e-7));
  }
  for(size_t i=0;i<ramps.size();i++) {
    assert(ramps[i].IsValid());
  }
  PiecewisePolynomialND poly;
  ParabolicRamp::ParabolicRampND castRamp;
  ToTrajClient(ramps[0],castRamp,this);
  poly = Cast(castRamp);
  for(size_t i=1;i<ramps.size();i++) {
    ToTrajClient(ramps[i],castRamp,this);
    poly.Concat(Cast(castRamp),true);
  }
  return client.AddTrajectoryImmediate(tstart,poly);
}


///basic queries
bool KlamptToVal3Interface::GetConfig(Config& q)
{
  TrajClient::Vector x;
  if(!client.GetConfig(x)) return false;
  ConfigFromTrajClient(x,q);
  return true;
}

bool KlamptToVal3Interface::GetVelocity(Vector& v)
{
  TrajClient::Vector x;
  if(!client.GetVelocity(x)) return false;
  VectorFromTrajClient(x,v);
  return true;
}

bool KlamptToVal3Interface::GetTransform(RigidTransform& T)
{
  TrajClient::Vector x;
  if(!client.GetTransform(x)) return false;
  double mat[16];
  VAL3TransformToMatrix(x,mat);
  T.set(Matrix4(mat));
  return true;
}

bool KlamptToVal3Interface::GetJointLimits(Config& jmin,Config& jmax)
{
  TrajClient::Vector xmin,xmax;
  if(!client.GetJointLimits(xmin,xmax)) return false;
  VectorFromTrajClient(xmin,jmin);
  VectorFromTrajClient(xmax,jmax);
  for(int i=0;i<jmin.n;i++)
    if(jmin(i) > jmax(i)) Swap(jmin(i),jmax(i));
  return true;

}
bool KlamptToVal3Interface::GetVelocityLimits(Vector& vmax)
{
  TrajClient::Vector x;
  if(!client.GetVelocityLimits(x)) return false;
  VectorFromTrajClient(x,vmax);
  for(int i=0;i<vmax.n;i++)
    vmax(i)=Abs(vmax(i));
  return true;
}

bool KlamptToVal3Interface::GetAccelerationLimits(Vector& amax)
{
  TrajClient::Vector x;
  if(!client.GetAccelerationLimits(x)) return false;
  VectorFromTrajClient(x,amax);
  for(int i=0;i<amax.n;i++)
    amax(i)=Abs(amax(i));
  return true;
}

bool KlamptToVal3Interface::GetDecelerationLimits(Vector& dmax)
{
  TrajClient::Vector x;
  if(!client.GetDecelerationLimits(x)) return false;
  VectorFromTrajClient(x,dmax);
  for(int i=0;i<dmax.n;i++)
    dmax(i)=Abs(dmax(i));
  return true;
}

bool KlamptToVal3Interface::GetEndConfig(Config& qend)
{
  TrajClient::Vector x;
  if(!client.GetEndConfig(x)) return false;
  ConfigFromTrajClient(x,qend);
  return true;
}

bool KlamptToVal3Interface::GetEndVelocity(Vector& vend)
{
  TrajClient::Vector x;
  if(!client.GetEndVelocity(x)) return false;
  VectorFromTrajClient(x,vend);
  return true;
}

///Sets new (more conservative) joint limits
void KlamptToVal3Interface::SetJointLimits(const Vector& qmin,const Vector& qmax)
{
  TrajClient::Vector xmin,xmax;
  ConfigToTrajClient(qmin,xmin);
  ConfigToTrajClient(qmax,xmax);
  return client.SetJointLimits(xmin,xmax);
}

///Sets new (more conservative) velocity limits
void KlamptToVal3Interface::SetSpeedScale(double rate)
{
  client.SetSpeedScale(rate);
}

