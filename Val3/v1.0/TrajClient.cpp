#include "TrajClient.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <time.h> 
#include <math.h> 
#include <string.h> 
#include <assert.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>

using namespace std;

char* AscLocalTime()
{
  time_t rawtime;
  struct tm * timeinfo;
  time ( &rawtime );
  timeinfo = localtime ( &rawtime );
  char* ret=asctime(timeinfo);
  //asctime returns a newline character at end of string
  ret[strlen(ret)-1]=0;
  return ret;
}

inline double ReadDouble(const string& s)
{
  double val;
  stringstream ss(s);
  ss>>val;
  assert(ss);
  return val;
}

inline int ReadInt(const string& s)
{
  int val;
  stringstream ss(s);
  ss>>val;
  assert(ss);
  return val;
}


template <class T>
inline double ReadVector(const string& s,vector<T>& v)
{
  v.resize(0);
  stringstream ss(s);
  T elem;
  while(true) {
    ss>>elem;
    if(ss.fail()) return true;
    if(ss.bad()) return false;
    v.push_back(elem);
  }
  return true;
}

template <class T>
inline string Join(const string& sep,const vector<T>& v)
{
  stringstream ss;
  for(size_t i=0;i<v.size();i++) {
    ss<<v[i];
    if(i+1 != v.size())
      ss<<sep;
  }
  return ss.str();
}

template <class T>
inline std::string ToString (const T& t)
{
  std::stringstream ss;
  ss << t;
  return ss.str();
}

const char* TrajClient::VERSION = "1.0";


TrajClient::TrajClient(const char* host,int port)
{
  this->sockfd = -1;
  this->name = "Unnamed";
  this->curId = 0;
  if (host != NULL)
    this->Connect(host,port);
}

TrajClient::~TrajClient()
{
  flog.close();
  if(sockfd >= 0)
    close(sockfd);
}

bool TrajClient::LogBegin(const char* fn)
{
  flog.close();
  flog.clear();
  flog.open(fn,ios::app);
  if(flog.fail()) return false;
  return true;
}

bool TrajClient::LogEnd()
{
  flog.close();
  return true;
}

bool TrajClient::Connect(const char* host,int port)
{
  if (this->name == "Unnamed") {
    stringstream ss;
    ss<<host<<":"<<port;
    this->name = ss.str();
  }
  struct sockaddr_in serv_addr;
  struct hostent *server;

  this->sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (this->sockfd < 0) {
    fprintf(stderr,"Error creating socket\n");
    return false;
  }
  server = gethostbyname(host);
  if (server == NULL) {
    fprintf(stderr,"ERROR, no such host %s:%d\n",host,port);
    close(sockfd);
    sockfd = -1;
    return false;
  }
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  memcpy(&serv_addr.sin_addr.s_addr,
	 server->h_addr,
	 server->h_length);
  serv_addr.sin_port = htons(port);
  if (connect(sockfd,(sockaddr*)&serv_addr,sizeof(serv_addr)) < 0) {
    fprintf(stderr,"Connect to %s:%d failed\n",host,port);
    perror("");
    close(sockfd);
    sockfd = -1;
    return false;
  }

  //connected! now check the version
  if (this->Version()!=string(VERSION)) {
    fprintf(stderr,"Connected to version %s of trajClient, need %s\n",this->Version().c_str(),VERSION);
    return false;
  }
  return true;
}

void TrajClient::Disconnect()
{
  if(sockfd >= 0)
    close(sockfd);
  sockfd = -1;
}

bool TrajClient::IsConnected() const
{
  return (sockfd >= 0);
}

int TrajClient::SendMessageRaw(const string& cmd,bool wantReply)
{
  int retval = -1;
  string msg;
  if(!wantReply)
    msg = "* ";
  else {
    retval = this->curId;
    stringstream ss;
    ss<<this->curId<<" ";
    msg = ss.str();
  }
  msg = msg+cmd+";";
            
  // Send the message
  if(this->flog) {
    this->flog << AscLocalTime() << ": " << this->name << " <- "<<msg<<endl;
  }

  int msglen = msg.length();
  int totalsent = 0;
  while(totalsent < msglen) {
	int n = write(this->sockfd,msg.c_str()+totalsent,msglen-totalsent);
	if(n <= 0) {
	  fprintf(stderr,"Error sending message %d\n",this->curId);
	  return -1;
	}
	totalsent += n;
  }
  this->curId = this->curId+1;
  return retval;
}

int TrajClient::SendMessageRaw(const vector<string>& cmds,bool wantReply)
{
  if(cmds.empty()) return -1;
  int retval = -1;
  string msg;
  if(!wantReply)
    msg = "* ";
  else {
    retval = this->curId;
    stringstream ss;
    ss<<this->curId<<" ";
    msg = ss.str();
  }
  for(size_t i=0;i<cmds.size();i++) {
    if(i+1 == cmds.size())
      msg += cmds[i]+";";
    else
      msg += cmds[i]+",";
  }

  // Send the message
  if(this->flog) {
    this->flog << AscLocalTime() << ": " << this->name << " <- "<<msg<<endl;
  }

  int msglen = msg.length();
  int totalsent = 0;
  while(totalsent < msglen) {
	int n = write(this->sockfd,msg.c_str()+totalsent,msglen-totalsent);
	if(n <= 0) {
	  fprintf(stderr,"Error sending message %d\n",this->curId);
	  return -1;
	}
	totalsent += n;
  }
  this->curId = this->curId+1;
  return retval;
}

string TrajClient::ReceiveReply(double delay)
{
  bool done = false;
  string msg = "";
  char buffer[1025];
  while(!done) {
    int n=read(this->sockfd,buffer,1024);
    if(n < 0) {
      fprintf(stderr,"Error receiving message %d\n",this->curId);
      return "Invalid";
    }
    //delete null characters
    int nchr=0;
    for(int i=0;i<n;i++)
      if(buffer[i]!=0)
	buffer[nchr++] = buffer[i];
    buffer[nchr]=0;
    msg += buffer;
    //check for completion
    if (msg.length() > 0 && msg[msg.length()-1]==';')
      done = true;
    //printf("Read partial %s, %d\n",msg.c_str(),msg.length());
    if (delay > 0 && !done) {
      usleep(int(delay*1000000.0));
    }
  }
  if(this->flog) {
    this->flog << AscLocalTime() << ": " << this->name << " -> " << msg <<endl;
  }
  return msg;
}

string TrajClient::SendMessage(const string& cmd,bool wantReply)
{
  //std::cout<<"Sent message "<<cmd<<std::endl; 
  int msgid = this->SendMessageRaw(cmd,wantReply);
        
  //Receive the reply
  if(wantReply) {
    string ret = this->ReceiveReply();
    //std::cout<<"Got reply "<<ret<<std::endl;
    size_t spaceindex = ret.find(" ");
    if(spaceindex == string::npos) {
      fprintf(stderr,"Error, return value doesn't have a valid index\n");
      return "Invalid";
    }
    if(ret[ret.length()-1]!=';') {
      fprintf(stderr,"Error, return value is not semicolon terminated\n");
      return "Invalid";
    }
    //split the string, take out semicolon
    string retid = ret.substr(0,spaceindex);
    string res = ret.substr(spaceindex+1,ret.length()-spaceindex-2);
    return res;
  }
  return "";
}

vector<string> TrajClient::SendMessage(const vector<string>& cmds,bool wantReply)
{
  int msgid = this->SendMessageRaw(cmds,wantReply);
        
  //Receive the reply
  vector<string> res;
  if(wantReply) {
    string ret = this->ReceiveReply();
    size_t spaceindex = ret.find(" ");
    if(spaceindex == string::npos) {
      fprintf(stderr,"Error, return value doesn't have a valid index\n");
      return res;
    }
    if(ret[ret.length()-1]!=';') {
      fprintf(stderr,"Error, return value is not semicolon terminated\n");
      return res;
    }
    //split the string, take out semicolon
    string retid = ret.substr(0,spaceindex);
    string results = ret.substr(spaceindex+1,ret.length()-spaceindex-2);
    //tokenize results by ','
    int pos=0;
    while(true) {
      size_t endpos = results.find(pos,',');
      if(endpos == string::npos) {
	res.push_back(results.substr(pos,results.length()-pos));
	break;
      }
      else {
	res.push_back(results.substr(pos,endpos-pos));
	pos = endpos+1;
      }
    }
  }
  return res;
}

string TrajClient::Call(const string& func,const string& args,bool wantReply)
{
  return this->SendMessage(func+"("+args+")",wantReply);
}

vector<string> TrajClient::Call(const vector<FunctionCall>& calls,bool wantReply)
{
  vector<string> msgs(calls.size());
  for(size_t i=0;i<calls.size();i++) {
    msgs[i] = calls[i].first+"("+calls[i].second+")";
  }
  return this->SendMessage(msgs,wantReply);
}

string TrajClient::Echo(const string& message)
{
  return Call("echo",message);
}

string TrajClient::Version()
{
  return Call("version","");
}


double TrajClient::Rate()
{
  return ReadDouble(Call("rate",""));
}

bool TrajClient::GetConfig(Vector& q)
{
  return ReadVector(Call("gj",""),q);
}

bool TrajClient::GetVelocity(Vector& v)
{
  return ReadVector(Call("gv",""),v);
}

bool TrajClient::GetTransform(Vector& t)
{
  return ReadVector(Call("gx",""),t);
}

bool TrajClient::GetJointLimits(Vector& jmin,Vector& jmax)
{
  return ReadVector(Call("gjmin",""),jmin) && ReadVector(Call("gjmax",""),jmax);
}

bool TrajClient::GetVelocityLimits(Vector& vmax)
{
  return ReadVector(Call("gvl",""),vmax);
}

bool TrajClient::GetAccelerationLimits(Vector& amax)
{
  return ReadVector(Call("gal",""),amax);
}

bool TrajClient::GetDecelerationLimits(Vector& dmax)
{
  return ReadVector(Call("gdl",""),dmax);
}

bool TrajClient::SetJointLimits(const Vector& jmin,const Vector& jmax)
{
  Call("sjmin",Join(" ",jmin));
  Call("sjmax",Join(" ",jmax));
  return true;
}

bool TrajClient::SetVelocityLimits(const Vector& vmax)
{
  Call("svl",Join(" ",vmax));
  return true;
}

bool TrajClient::SetAccelerationLimits(const Vector& amax)
{
  Call("sal",Join(" ",amax));
  return true;
}

bool TrajClient::SetDecelerationLimits(const Vector& dmax)
{
  Call("sdl",Join(" ",dmax));
  return true;
}

int TrajClient::GetMaxSegments()
{
  return ReadInt(Call("gms",""));
}

int TrajClient::GetCurSegments()
{
  return ReadInt(Call("gcs",""));
}

int TrajClient::GetRemainingSegments()
{
  vector<FunctionCall> funcs(2);
  funcs[0].first = "gms";
  funcs[1].first = "gcs";
  vector<string> res=Call(funcs);
  return ReadInt(res[0])-ReadInt(res[1]);
}

double TrajClient::GetCurrentTime()
{
  return ReadDouble(Call("gct",""));
}

double TrajClient::GetTrajEndTime()
{
  return ReadDouble(Call("get",""));
}

double TrajClient::GetTrajDuration()
{
  return ReadDouble(Call("gd",""));
}

bool TrajClient::GetEndConfig(Vector& jend)
{
  return ReadVector(Call("gej",""),jend);
}

bool TrajClient::GetEndVelocity(Vector& vend)
{
  return ReadVector(Call("gev",""),vend);
}

int TrajClient::AddMilestone(double dt,const Vector& j)
{
  string res = Call("am",ToString(dt)+" "+Join(" ",j));
  if(res == "Error") {
    fprintf(stderr,"AddMilestone failed");
    abort();
  }
  return ReadInt(res);
}

bool TrajClient::AddMilestoneQuiet(double dt,const Vector& j)
{
  Call("am",ToString(dt)+" "+Join(" ",j),false);
  return true;
}

vector<int> TrajClient::AppendMilestones(const vector<double>& dts,const vector<Vector>& q)
{
  assert(dts.size()==q.size());
  vector<FunctionCall> funcs;
  funcs.resize(dts.size());
  for(size_t i=0;i<dts.size();i++) {
    assert(dts[i] > 0);
    funcs[i] = FunctionCall("am",ToString(dts[i])+" "+Join(" ",q[i]));
  }
  vector<string> res=Call(funcs);
  vector<int> indices(res.size());
  for(size_t i=0;i<res.size();i++) {
    if(res[i] == "Error") {
      fprintf(stderr,"AppendMilestone failed");
      abort();
    }
    indices[i] = ReadInt(res[i]);
  }
  return indices;
}

bool TrajClient::AppendMilestonesQuiet(const vector<double>& dts,const vector<Vector>& q)
{
  assert(dts.size()==q.size());
  vector<FunctionCall> funcs;
  funcs.resize(dts.size());
  for(size_t i=0;i<dts.size();i++) {
    assert(dts[i] > 0);
    funcs[i] = FunctionCall("am",ToString(dts[i])+" "+Join(" ",q[i]));
  }
  Call(funcs,false);
  return true;
}

bool TrajClient::ResetTrajectoryAbs(double t)
{
  string res=Call("rtabs",ToString(t));
  if(res=="Error")
    return false;
  return true;
}

bool TrajClient::ResetTrajectoryRel(double dt)
{
  string res=Call("rtrel",ToString(dt));
  if(res=="Error")
    return false;
  return true;
}

bool TrajClient::CheckTrajectory()
{
  string res=Call("check","");
  if(res=="")
    return true;
  else {
    fprintf(stderr,"Trajectory infeasible: %s",res.c_str());
    return false;
  }
}




void MMult3x3(const double a[9],const double b[9],double out[9])
{
  for(int i=0;i<3;i++) {
    for(int j=0;j<3;j++) {
      double res=0;
      for(int k=0;k<3;k++)
	res += a[k*3+i]*b[j*3+k];
      out[j*3+i] = res;
    }
  }
}

void RotationMatrix3x3(double x,double y,double z,double rads,double mat[9])
{
  double c = cos(rads);
  double s = sin(rads);
  double val [9] = {c,0.0,0.0, 0.0,c,0.0, 0.0,0.0,c};
  double rrt [9] = {x*x,x*y,x*z, y*x,y*y,y*z, z*x,z*y,z*z};
  double cross [9] = {0.0,z,-y, -z,0.0,x, y,-x,0.0};
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      mat[j*3+i] = val[j*3+i] + (1.0-c)*rrt[j*3+i] + s*cross[j*3+i];
}

void VAL3TransformToMatrix(const std::vector<double>& Tval3,double mat[16])
{
  assert(Tval3.size()==6);
  mat[0*4+3] = 0.0;
  mat[1*4+3] = 0.0;
  mat[2*4+3] = 0.0;
  //translation
  mat[3*4+0] = Tval3[0]*0.001;
  mat[3*4+1] = Tval3[1]*0.001;
  mat[3*4+2] = Tval3[2]*0.001;
  mat[3*4+3] = 1.0;

  double rx = Tval3[4]*M_PI/180.0;
  double ry = Tval3[5]*M_PI/180.0;
  double rz = Tval3[6]*M_PI/180.0;
  //rotations applied as rx ry rz in that order;
  double Rx[9];
  double Ry[9];
  double Rz[9];
  double temp[9],temp2[9];
  RotationMatrix3x3(1.0,0.0,0.0, rx,Rx);
  RotationMatrix3x3(0.0,1.0,0.0, ry,Ry);
  RotationMatrix3x3(0.0,0.0,1.0, rz,Rz);
  MMult3x3(Rx,Ry,temp);
  MMult3x3(temp,Rz,temp2);
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      mat[j*4+i] = temp2[j*3+i];
}
