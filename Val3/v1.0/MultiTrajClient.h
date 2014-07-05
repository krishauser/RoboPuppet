#ifndef MULTI_TRAJ_CLIENT_H
#define MULTI_TRAJ_CLIENT_H

#include "TrajServer.h"

/**
 * A class that connects to multiple trajectory clients at once and that
 * can interleave RPC calls for better motion synchronization.
 *
 * The standard convention is that all methods in TrajClient are duplicated
 * here, except with an additional argument 'h' that defines the host index.
 *
 * For several methods, the methods without 'h' use a specially synch'ed
 * version of the call.
 *
 * For example, MultiTrajClient.GetConfig(qs) will return configurations that
 * are more likely to have occurred at the same time than calling GetConfig(h)
 * for h=0...N (where N is the maximum index of the trajClients).
 *
 * WARNING: not yet tested!
 */
class MultiTrajClient
{
 public:
  typedef TrajClient::Vector Vector;
  typedef TrajClient::Command Command;
  typedef TrajClient::Reply Reply;
  typedef TrajClient::MultiCommand MultiCommand;
  typedef TrajClient::MultiReply MultiReply;
  typedef TrajClient::FunctionCall FunctionCall;
  typedef TrajClient::MultiFunctionCall MultiFunctionCall;

  MultiTrajClient(const std::vector<std::string>& hosts,int port=1000);
  MultiTrajClient(const std::vector<std::string>& hosts,const std::vector<int>& ports);

  bool LogBegin(const char* fn="trajclient.log");
  bool LogEnd();

  bool Connect(const std::vector<std::string>& hosts,int port=1000);
  bool Connect(const std::vector<std::string>& hosts,const std::vector<int>& ports);

  /** Sends cmds to only server h.  See TrajClient.sendMessage */
  Reply SendMessage(int h,const Command& cmd,bool wantReply=true);
  MultiReply SendMessage(int h,const MultiCommand& cmd,bool wantReply=true);

  /**
   * Sends the commands in cmds[i] to controller i.  This is essentially
   * the same as:
   *
   * for(i=0;i<cmds.size();i++)
   *        this->SendMessage(i,cmds[i])
   *
   * except all messages are sent at once, and then all return values are
   * received at once, which may be better synchronized.
   */      
  std::vector<Reply> SendMessageAll(const vector<Command>& cmds,bool wantReply=true);
  std::vector<MultiReply> SendMessageAll(const std::vector<MultiCommand>& cmds,bool wantReply=true);

  Reply Call(int h,const std::string& func,const std::string& args,bool wantReply=true);
  MultiReply Call(int h,const std::vector<FunctionCall>& calls,bool wantReply=true);

  std::vector<Reply> CallAll(const std::vector<std::string>& func,const std::vector<std::string>& args,bool wantReply=true);
  std::vector<MultiReply> CallAll(const std::vector<MultiFunctionCall>& funcs,bool wantReply=true);

  /**
   * Sends the command(s) func(args) to all servers, returns the
   * list of return values.
   */
  std::vector<Reply> BroadcastCall(const std::string& func,const std::string& args);

  std::string Echo(int h,const std::string& message);
  std::string Version(int h);
  double Rate(int h) { return clients[h].Rate(); }
  bool GetConfig(int h,Vector& q) { return clients[h].GetConfig(q); }
  bool GetVelocity(int h,Vector& v) { return clients[h].GetVelocity(v); }
  bool GetTransform(int h,Vector& t) { return clients[h].GetTransform(t); }
  bool GetJointLimits(int h,Vector& jmin,Vector& jmax) { return clients[h].GetJointLimits(jmin,jmax); }
  bool GetVelocityLimits(int h,Vector& vmax) { return clients[h].GetVelocityLimits(vmax); }
  bool GetAccelerationLimits(int h,Vector& amax) { return clients[h].GetAccelerationLimits(amax); }
  bool GetDecelerationLimits(int h,Vector& dmax) { return clients[h].GetDecelerationLimits(dmax); }

  bool SetJointLimits(int h,const Vector& jmin,const Vector& jmax) { return clients[h].SetJointLimits(jmin,jmax); }
  bool SetVelocityLimits(int h,const Vector& vmax) { return clients[h].SetVelocityLimits(vmax); }
  bool SetAccelerationLimits(int h,const Vector& amax) { return clients[h].SetAccelerationLimits(amax); }
  bool SetDecelerationLimits(int h,const Vector& dmax) { return clients[h].SetDecelerationLimits(dmax); }

  int GetMaxSegments(int h) { return clients[h].GetMaxSegments(); }
  int GetCurSegments(int h) { return clients[h].GetCurSegments(); }
  int GetRemainingSegments(int h) { return clients[h].GetRemainingSegments(); }
  double GetCurrentTime(int h) { return clients[h].GetCurrentTime(); }
  double GetTrajEndTime(int h) { return clients[h].GetTrajEndTime(); }
  double GetTrajDuration(int h) { return clients[h].GetTrajDuration(); }
  bool GetEndConfig(int h,Vector& jend) { return clients[h].GetEndConfig(jend); }
  bool GetEndVelocity(int h,Vector& vend) { return clients[h].GetEndVelocity(vend); }

  int AddMilestone(int h,double dt,const Vector& j) { return clients[h].AddMilestone(dt,j); }
  bool AddMilestoneQuiet(int h,double dt,const Vector& j) { return clients[h].AddMilestoneQuiet(dt,j); }

  std::vector<int> AppendMilestones(int h,const std::vector<double>& dts,const std::vector<Vector>& q) { return clients[h].AppendMilestones(dts,q); }
  bool AppendMilestonesQuiet(int h,const std::vector<double>& dts,const std::vector<Vector>& q) { return clients[h].AppendMilestonesQuiet(dts,q); }

  bool ResetTrajectoryAbs(int h,double t) { return clients[h].ResetTrajectoryAbs(t); }
  bool ResetTrajectoryRel(int h,double dt) { return clients[h].ResetTrajectoryRel(dt); }
  bool CheckTrajectory(int h) { return clients[h].CheckTrajectory(); }

  /** The following support synchronized broadcast calling **/
  bool GetConfig(const std::vector<Vector>& q);
  bool GetVelocity(const std::vector<Vector>& v);
  bool GetTransform(const std::vector<Vector>& t);
  bool GetCurrentTime(const std::vector<double>& q);
  bool GetTrajDuration(const std::vector<double>& q);

  std::vector<int> AddMilestoneAll(const std::vector<double> dts,const std::vector<Vector>& qs);
  std::vector<int> AppendMilestonesAll(const std::vector<std::vector<double> > dts,const std::vector<std::vector<Vector> >& qs);

  std::vector<TrajClient> clients;
};

/**
 * A multi-trajectory-client that acts in the combined state space of
 * all robots.
 *
 * WARNING: not yet tested!
 */
class UnifiedTrajClient
{
 public:
  UnifiedTrajClient(const std::vector<std::string>& hosts,int port=1000);
  UnifiedTrajClient(const std::vector<std::string>& hosts,std::vector<int>& ports);
  
  std::string Echo(const std::string& message);
  std::string Version();
  double Rate();
  bool GetConfig(Vector& q);
  bool GetVelocity(Vector& v);
  bool GetTransforms(std::vector<Vector>& t);
  bool GetJointLimits(Vector& jmin,Vector& jmax);
  bool GetVelocityLimits(Vector& vmax);
  bool GetAccelerationLimits(Vector& amax);
  bool GetDecelerationLimits(Vector& dmax);

  bool SetJointLimits(const Vector& jmin,const Vector& jmax);
  bool SetVelocityLimits(const Vector& vmax);
  bool SetAccelerationLimits(const Vector& amax);
  bool SetDecelerationLimits(const Vector& dmax);

  int GetMaxSegments();
  int GetCurSegments();
  int GetRemainingSegments();
  double GetCurrentTime();
  double GetTrajEndTime();
  double GetTrajDuration();
  bool GetEndConfig(Vector& jend);
  bool GetEndVelocity(Vector& vend);

  int AddMilestone(double dt,const Vector& j);
  bool AddMilestoneQuiet(double dt,const Vector& j);
  std::vector<int> AppendMilestones(const std::vector<double>& dts,const std::vector<Vector>& q);
  bool AppendMilestonesQuiet(const std::vector<double>& dts,const std::vector<Vector>& q);

  bool ResetTrajectoryAbs(double t);
  bool ResetTrajectoryRel(double dt);
  bool CheckTrajectory();

 private:
  MultiTrajClient client;
  std::vector<double> toffsets;
};

#endif
