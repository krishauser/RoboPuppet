#ifndef VAL3_TRAJECTORY_CLIENT_H
#define VAL3_TRAJECTORY_CLIENT_H

#include <string>
#include <vector>
#include <fstream>

/**
 * An implementation of the VAL3 trajectory client API to be used
 * with the trajServer program.
 *
 * See the accompanying documentation for information about the
 * commands and motion queue semantics. 
 */
class TrajClient
{
 public:
  typedef std::vector<double> Vector;
  typedef std::string Command;
  typedef std::string Reply;
  typedef std::vector<Command> MultiCommand;
  typedef std::vector<Reply> MultiReply;
  typedef std::pair<std::string,std::string> FunctionCall;
  typedef std::vector<FunctionCall> MultiFunctionCall;

  TrajClient(const char* host=NULL,int port=1000);
  ~TrajClient();

  bool LogBegin(const char* fn="trajclient.log");
  bool LogEnd();

  bool Connect(const char* host,int port=1000);
  void Disconnect();
  bool IsConnected() const;

  /**
   * Sends the commands cmds, which may be one or more function calls.
   * Does not wait for a return value from the server.
   * The method will return the ID of the expected return message, or -1
   * if no message will be returned.
   *
   * Warning: if wantReply is true (it is by default), then ReceiveReply()
   * must be called before sending additional messages!
   **/
  int SendMessageRaw(const Command& cmd,bool wantReply=true);
  int SendMessageRaw(const MultiCommand& cmds,bool wantReply=true);

  /**
   * Returns the entire reply message without parsing.
   * Warning: this must only be called when awaiting a reply to an earlier
   * message.  Otherwise, it will block and never return.
   */
  Reply ReceiveReply(double delay=0);

  /**
   * Sends the command strings cmds, which may be one or more function calls.
   * Returns the list of return values.
   */
  Reply SendMessage(const Command& cmd,bool wantReply=true);
  MultiReply SendMessage(const MultiCommand& cmds,bool wantReply=true);

  /**
   * Calls the functions in funcs with corresponding arguments in args
   * and returns a list of return values.
   *
   * Can be called using a list of functions or just a single function.
   * In the latter case, just returns the return value.
   */
  Reply Call(const std::string& func,const std::string& args,bool wantReply=true);
  MultiReply Call(const MultiFunctionCall& calls,bool wantReply=true);

  std::string Echo(const std::string& message);
  std::string Version();
  double Rate();
  bool GetConfig(Vector& q);
  bool GetVelocity(Vector& v);
  bool GetTransform(Vector& t);
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

  /**
   * Appends the milestone q to be reached at the time dt after the
   * end of the motion queue (or the time the command is received, if
   * the motion queue is currently empty).  The robot will perform
   * a linear interpolation in joint space from the prior configuration
   * to q.
   *
   * Returns the milestone index.
   */
  int AddMilestone(double dt,const Vector& j);
  bool AddMilestoneQuiet(double dt,const Vector& j);

  /**
   * Appends milestones in q to the end of the trajectory queue,
   * separated respectively by the times in dts.  Returns the
   * indices of the resulting segments.
   */
  std::vector<int> AppendMilestones(const std::vector<double>& dts,const std::vector<Vector>& q);
  bool AppendMilestonesQuiet(const std::vector<double>& dts,const std::vector<Vector>& q);


  /** Resets the trajectory at absolute (controller) time t. */
  bool ResetTrajectoryAbs(double t);
  /**
   * Resets the trajectory at time dt after the instant that the call is
   * received.
   */
  bool ResetTrajectoryRel(double dt);
  /** Performs a trajectory check on the controller. */
  bool CheckTrajectory();

  static const char* VERSION;

  std::string name;
  int curId;
  int sockfd;
  std::ofstream flog;
};


/**
 * A 'sanitized' version of the TrajClient.  No movement-generating
 * commands are allowed in order to help prevent accidents.
 */
class ReadOnlyTrajClient
{
 public:
  typedef TrajClient::Vector Vector;
  
  ReadOnlyTrajClient(TrajClient& _client) : client(_client) {}
  std::string Echo(const std::string& message) { return client.Echo(message); }
  std::string Version() { return client.Version(); }
  double Rate() { return client.Rate(); }
  bool GetConfig(Vector& q) { return client.GetConfig(q); }
  bool GetVelocity(Vector& v) { return client.GetVelocity(v); }
  bool GetTransform(Vector& t) { return client.GetTransform(t); }
  bool GetJointLimits(Vector& jmin,Vector& jmax) { return client.GetJointLimits(jmin,jmax); }
  bool GetVelocityLimits(Vector& vmax) { return client.GetVelocityLimits(vmax); }
  bool GetAccelerationLimits(Vector& amax) { return client.GetAccelerationLimits(amax); }
  bool GetDecelerationLimits(Vector& dmax) { return client.GetDecelerationLimits(dmax); }
  int GetMaxSegments() { return client.GetMaxSegments(); }
  int GetCurSegments() { return client.GetCurSegments(); }
  int GetRemainingSegments() { return client.GetRemainingSegments(); }
  double GetCurrentTime() { return client.GetCurrentTime(); }
  double GetTrajEndTime() { return client.GetTrajEndTime(); }
  double GetTrajDuration() { return client.GetTrajDuration(); }
  bool GetEndConfig(Vector& jend) { return client.GetEndConfig(jend); }
  bool GetEndVelocity(Vector& vend) { return client.GetEndVelocity(vend); }
  bool CheckTrajectory()  { return client.CheckTrajectory(); }

 private:
  TrajClient& client;
};

/** Converts the VAL3 transformation (result of GetTransform) to a 4x4
 * homogeneous matrix transformation.  Matrix addressing is column major.
 *
 * Converts mm to meters, and converts degrees to radians.
 */
void VAL3TransformToMatrix(const std::vector<double>& Tval3,double mat[16]);

#endif
