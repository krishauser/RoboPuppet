#include "TrajServerEmulator.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <time.h> 
#include <math.h> 
#include <string.h> 
#include <assert.h>
#include <stdlib.h>
#include <sys/unistd.h>
#include <sys/fcntl.h>
#include <errno.h>
#include <sstream>
#include <iostream>
using namespace std;
#ifndef SOCKET_ERROR
#define SOCKET_ERROR -1
#endif

const static size_t kMaxClients = 4;

typedef vector<double> Vector;

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
inline bool ReadVector(const string& s,vector<T>& v)
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

void Split(const string& str, const string& delim, vector<string>& parts)
{
  parts.resize(0);
  size_t start, end = 0;
  while (end < str.size()) {
    start = end;
    while (start < str.size() && (delim.find(str[start]) != string::npos)) {
      start++;  // skip initial whitespace
    }
    end = start;
    while (end < str.size() && (delim.find(str[end]) == string::npos)) {
      end++; // skip to end of word
    }
    if (end-start != 0) {  // just ignore zero-length strings.
      parts.push_back(string(str, start, end-start));
    }
  }
}


template <class T>
inline std::string ToString (const T& t)
{
  std::stringstream ss;
  ss << t;
  return ss.str();
}


TrajServerEmulator::TrajServerEmulator(CS8Emulator& _emulator)
  :emulator(_emulator),sockserv(-1),nReceived(0),nSent(0),nCalls(0),
   simulatedLatency(0)
{}

TrajServerEmulator::~TrajServerEmulator()
{
  Cleanup();
}


bool TrajServerEmulator::Init(int recvport)
{
  struct sockaddr_in sock_addr;

  sockserv = socket(AF_INET,SOCK_STREAM | SOCK_NONBLOCK,0);
  //sockserv = socket(AF_INET,SOCK_STREAM,0);

  if(sockserv == SOCKET_ERROR)
  {
    printf("Could not make a socket\n");
    return false;
  }

  /* fill address struct */
  sock_addr.sin_addr.s_addr=INADDR_ANY;
  sock_addr.sin_port=htons(recvport);
  sock_addr.sin_family=AF_INET;


  /* bind to a port */
  if(bind(sockserv,(struct sockaddr*)&sock_addr,sizeof(sock_addr)) == SOCKET_ERROR) {
    fprintf(stderr,"Bind server to %d failed\n",recvport);
    perror("");
    close(sockserv);
    sockserv = -1;
    return false;
  }
  return true;
}

void TrajServerEmulator::Cleanup()
{
  CloseConnections();
  if(sockserv != -1) {
    close(sockserv);
    sockserv = -1;
  }
}

bool TrajServerEmulator::AcceptConnection()
{
  struct sockaddr_in sock_addr;
  if(sockrecv.size() >= kMaxClients) {
    fprintf(stderr,"%d client connections already active\n",kMaxClients);
    return false;
  }
  /*  get port number */
  socklen_t len = sizeof(sock_addr);
  getsockname( sockserv, (struct sockaddr *) &sock_addr,(socklen_t *)&len);

  /* establish listen queue */
  if(listen(sockserv,1) == SOCKET_ERROR) {
    fprintf(stderr,"Listen failed\n");
    perror("");
    return false;
  }

  /* get the connected socket */
  int sockfd=accept(sockserv,(struct sockaddr*)&sock_addr,(socklen_t *)&len);
  if(sockfd < 0) {
    if(errno != EWOULDBLOCK) 
      perror("");
    return false;
  }
  fcntl(sockfd, F_SETFL, O_NONBLOCK);
  sockrecv.push_back(sockfd);
  partialInput.resize(partialInput.size()+1);
  return true;
}

bool TrajServerEmulator::CloseConnections()
{
  for(size_t i=0;i<sockrecv.size();i++) {
    close(sockrecv[i]);
  }
  sockrecv.resize(0);
  partialInput.resize(0);
  return true;
}


bool TrajServerEmulator::CloseConnection(int index)
{
  assert(index >= 0 && index < (int)sockrecv.size());
  close(sockrecv[index]);
  sockrecv[index]=sockrecv.back();
  sockrecv.resize(sockrecv.size()-1);
  partialInput[index] = partialInput.back();
  partialInput.resize(partialInput.size()-1);
  return true;
}

bool SendAll(int sockfd,const string& msg)
{
  int msglen = (int)msg.length();
  int totalsent = 0;
  while(totalsent < msglen) {
    int n=write(sockfd,msg.c_str()+totalsent,msglen-totalsent);
    if(n <= 0) {
      return false;
    }
    totalsent += n;
  }
  return true;
}

void TrajServerEmulator::Process()
{
  //advance time
  emulator.AdvanceTime(virtualTimer.ElapsedTime());


  //handle IO
  if(sockrecv.size() < kMaxClients) {
    if(AcceptConnection()) 
      printf("Accepted new connection\n");
  }
  int closed=-1;
  for(size_t c=0;c<sockrecv.size();c++) {
    vector<string> msgs;
    if(!PollRecv((int)c,msgs)) {
      fprintf(stderr,"Unable to receive message, killing connection\n");
      CloseConnection(c);
      closed = c;
      c--;
      break;
    }
    for(size_t i=0;i<msgs.size();i++) {
      if(!ParseMessage((int)c,msgs[i])) {
	fprintf(stderr,"Unable to process message, killing connection\n");
	CloseConnection(c);
	closed = c;
	c--;
	break;
      }
    }
  }
  double t=virtualTimer.ElapsedTime();
  for(size_t i=0;i<sendQueue.size();i++) {
    if(sendQueue[i].client == closed) {
      sendQueue.erase(sendQueue.begin());
      i--;
      continue;
    }
    if(sendQueue[i].client > this->sockrecv.size()) {
      sendQueue[i].client = closed;
      if(closed < 0) {
	sendQueue.erase(sendQueue.begin());
	i--;
	continue;
      }
    }
    if(sendQueue[i].sendTime <= t) {
      if(!SendAll(this->sockrecv[sendQueue[i].client],sendQueue[i].message)) {
	fprintf(stderr,"Unable to send reply message to client\n");
	sendQueue.erase(sendQueue.begin());
	i--;
	break;
      }
      sendQueue.erase(sendQueue.begin());
      i--;
    }
  }
}

bool TrajServerEmulator::PollRecv(int c,std::vector<std::string>& messages)
{
  assert(c >= 0 && c < (int)sockrecv.size());
  assert(sockrecv.size() == partialInput.size());
  messages.resize(0);
 
  char buffer[1500];
  int n=read(this->sockrecv[c],buffer,1500);

  if(n == 0) {
    fprintf(stderr,"Read returned 0, client must have died\n");
    return false;
  }
  else if(n < 0) {
    if(errno != EWOULDBLOCK) {
      perror("read()");
      return false;
    }
    else return true;
  }
  string& partial = this->partialInput[c];
  for(int i=0;i<n;i++)
    if(buffer[i] != 0)
      partial += buffer[i];

  int last = 0;
  for(size_t i=0;i<partial.length();i++) {
    if (partial[i] == ';') {
      string msg=partial.substr(last,i-last+1);
      messages.push_back(msg);
      last = i+1;
    }
  }
  partial = partial.substr(last,partial.length()-last);
  return true;
}


bool TrajServerEmulator::ParseMessage(int c,const std::string& str)
{
  assert(c >= 0 && c <(int)sockrecv.size());
  std::string id,cmd,args;
  size_t spaceindex = str.find(" ");
  if(spaceindex == string::npos) {
    fprintf(stderr,"Error, string value doesn't have a valid index\n");
    return false;
  }
  if(str[str.length()-1]!=';') {
    fprintf(stderr,"Error, string value is not semicolon terminated\n");
    return false;
  }
  id = str.substr(0,spaceindex);
  string rest = str.substr(spaceindex+1,str.length()-spaceindex-2);
  //parse out comma-separated calls
  string reply=id+" ";
  vector<string> messages;
  Split(rest,",",messages);
  if(messages.empty()) {
    fprintf(stderr,"Parse error for message '%s'\n",str.c_str());
    return false;
  }
  assert(messages.size() >= 1);
  for(size_t i=0;i<messages.size();i++) {
    size_t lparen=messages[i].find('(');
    if(lparen == string::npos || messages[i][messages[i].length()-1] != ')') {
      fprintf(stderr,"Error, call is not formatted correctly\n");
      cout<<messages[i]<<endl;
      return false;
    }
    cmd = messages[i].substr(0,lparen);
    args = messages[i].substr(lparen+1,messages[i].length()-lparen-2);
    if(i > 0) reply += ",";
    reply += DispatchCommand(cmd,args);
  }
  reply += ";";
  if(id != "*") {
    //requires a reply message
    //place on the send queue
    if(simulatedLatency == 0) { //send immediate
      if(!SendAll(this->sockrecv[c],reply)) {
	fprintf(stderr,"Unable to send reply message\n");
	return false;
      }
    }
    else {
      double t = virtualTimer.ElapsedTime();
      OutboundMessage msg;
      double jitterAmount = 2.0*double(rand())/double(RAND_MAX)-1.0;
      msg.sendTime = t+simulatedLatency+jitterAmount*simulatedJitter;
      if(sendQueue.empty()) msg.sendTime = std::max(msg.sendTime,t);
      else msg.sendTime = std::max(msg.sendTime,sendQueue.back().sendTime);
      msg.client = c;
      msg.message = reply;
      sendQueue.push_back(msg);
    }
  }
  return true;
}

std::string TrajServerEmulator::DispatchCommand(const std::string& cmd,const std::string& args)
{
  if(cmd == "echo") return args;
  else if(cmd == "version") return "1.0";
  else if(cmd == "rate") return ToString(emulator.Rate());
  else if(cmd == "check") {
    if(emulator.CheckTrajectory()) return "";
    else {
      fprintf(stderr,"TrajServerEmulator: check returned failure\n");
      return "Failed check";
    }
  }
  else if(cmd == "am") {
    stringstream ss(args);
    double dt;
    Vector j(6);
    ss >> dt;
    for(size_t i=0;i<6;i++)
      ss >> j[i];
    if(!ss) return "ParseError";
    int n=emulator.AddMilestone(dt,j);
    if(n < 0) {
      fprintf(stderr,"TrajServerEmulator: am returned an error\n");
      return "Error";
    }
    else return ToString(n);
  }
  else if(cmd == "gct") { return ToString(emulator.GetCurrentTime()); }
  else if(cmd == "gd") { return ToString(emulator.GetTrajDuration()); }
  else if(cmd == "get") { return ToString(emulator.GetTrajEndTime()); }
  else if(cmd == "gej") { 
    Vector v;
    emulator.GetEndConfig(v);
    return Join(" ",v);
  }
  else if(cmd == "gev") {
    Vector v;
    emulator.GetEndVelocity(v);
    return Join(" ",v);
  }
  else if(cmd == "gms") { return ToString(emulator.GetMaxSegments()); }
  else if(cmd == "gcs") { return ToString(emulator.GetCurSegments()); }
  else if(cmd == "rtabs") { 
    printf("Reset(abs) at time %g, current time %g\n",ReadDouble(args),emulator.GetCurrentTime());
    /* printf("Resetting at time %g, current time %g\n",ReadDouble(args),emulator.GetCurrentTime());
    printf("Config: ");
    Vector q;
    emulator.GetTrajConfig(ReadDouble(args),q);
    for(size_t i=0;i<q.size();i++)
      printf("%g ",q[i]);
    printf("\n");
    */
    if(emulator.ResetTrajectoryAbs(ReadDouble(args))) return "";
    else {
      fprintf(stderr,"TrajServerEmulator: rtabs returned an error\n");
      return "Error";
    }
  }
  else if(cmd == "rtrel") {
    printf("Reset(rel) at time %g, current time %g\n",ReadDouble(args),emulator.GetCurrentTime());
    if(emulator.ResetTrajectoryRel(ReadDouble(args))) return "";
    else {
      fprintf(stderr,"TrajServerEmulator: rtrel returned an error\n");
      return "Error";
    }
  }
  else if(cmd == "gj") {
    Vector v;
    emulator.GetConfig(v);
    return Join(" ",v);
  }
  else if(cmd == "gx") {
    Vector v;
    emulator.GetTransform(v);
    return Join(" ",v);
  }
  else if(cmd == "gv") {
    Vector v;
    emulator.GetVelocity(v);
    return Join(" ",v);
  }
  else if(cmd == "gjmin") { return Join(" ",emulator.qmin); }
  else if(cmd == "gjmax") { return Join(" ",emulator.qmax); }
  else if(cmd == "gvl") { return Join(" ",emulator.vmax); }
  else if(cmd == "gal") { return Join(" ",emulator.amax); }
  else if(cmd == "gdl") { return Join(" ",emulator.dmax); }
  else if(cmd == "sjmin") {
    Vector j;
    if(!ReadVector(args,j)) {
      fprintf(stderr,"TrajServerEmulator: sjmin returned an error\n");
      return "Error";
    }
    emulator.SetJointLimits(j,emulator.qmax);
  }
  else if(cmd == "sjmax") {
    Vector j;
    if(!ReadVector(args,j)) {
      fprintf(stderr,"TrajServerEmulator: sjmax returned an error\n");
      return "Error";
    }
    emulator.SetJointLimits(emulator.qmin,j);
  }
  else if(cmd == "svl") {
    Vector j;
    if(!ReadVector(args,j)) {
      fprintf(stderr,"TrajServerEmulator: svl returned an error\n");
      return "Error";
    }
    emulator.SetVelocityLimits(j);
  }
  else if(cmd == "sal") {
    Vector j;
    if(!ReadVector(args,j)) {
      fprintf(stderr,"TrajServerEmulator: sal returned an error\n");
      return "Error";
    }
    emulator.SetAccelerationLimits(j);
  }
  else if(cmd == "sdl") {
    Vector j;
    if(!ReadVector(args,j)) {
      fprintf(stderr,"TrajServerEmulator: sdl returned an error\n");
      return "Error";
    }
    emulator.SetDecelerationLimits(j);
  }
  else {
    fprintf(stderr,"Invalid command %s\n",cmd.c_str());
    return "Invalid";
  }
  return "";
}
