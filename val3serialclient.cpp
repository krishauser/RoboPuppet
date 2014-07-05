#include "Interface/UserInterface.h"
#include "Val3/Val3RobotInterface.h"
#include "Main/WorldViewProgram.h"
#include <utils/AnyCollection.h>
#include <GL/glui.h>
#include <GLdraw/GLUTString.h>
#include <fstream>
using namespace Math3D;
using namespace GLDraw;

enum {
  UI_LISTBOX_ID
};

//planner update time step
double dt=0.01;
//coefficient for the path duration cost in RealTimePlannerBase
double timeCostCoeff = 0.0;





class Val3SafeSerialProgram : public WorldViewProgram
{
public:
  WorldPlannerSettings plannerSettings;
  AnyCollection settings;

  KlamptToVal3Interface val3;
  Val3MotionQueueInterface robotInterface;
  vector<SmartPointer<RobotUserInterface> > uis;
  SmartPointer<InputProcessorBase> serialInputProcessor;
  int currentUI,oldUI;

  //GUI state
  GLUI* glui;
  GLUI_Listbox* ui_listbox;

  Val3SafeSerialProgram(RobotWorld* world)
    :WorldViewProgram(world),robotInterface(val3)
  {
    //setup the settings
    ifstream in("val3serialclient.settings",ios::in);
    bool readsettings = false;
    if(in) {
      in>>settings;
      if(in) 
	readsettings = true;
      else
	cerr<<"Error reading settings from val3serialclient.settings"<<endl;
    }
    if(!readsettings) {
      fprintf(stderr,"Need val3serialclient.settings file, copy and paste the following lines into the file.\n");
      settings = AnyCollection();
      settings["objective_address"]=string("tcp://localhost:3456");
      settings["objective_filter"]=string("objective");
      settings["val3server_setup"]=string("Val3/tx90right.xml");
      settings["val3server_address"]=string("localhost");
      settings["val3server_port"]=string("1000");
      settings["collision_margin"]=0.01;
      cerr<<settings<<endl;
      exit(-1);
    }
    string objsubaddr = settings["objective_address"];
    string objfilter = settings["objective_filter"];
    string val3setup = settings["val3server_setup"];
    string val3address = settings["val3server_address"];
    int val3port = int(settings["val3server_port"]);
    SocketObjectiveProcessor* processor = new SocketObjectiveProcessor(objsubaddr.c_str());
    serialInputProcessor = processor;
    if(!val3.LoadSetup(val3setup.c_str())) {
      FatalError("Error loading Val3 setup file %s\n",val3setup.c_str());
    }
    //comment this out if you don't want to log to trajclient.log
    val3.client.LogBegin();
    if(!val3.Connect(val3address.c_str(),val3port)) {
      FatalError("Error connecting to Val3 server %s:%d\n",val3address.c_str(),val3port);
    }
  }


  virtual bool Initialize()
  {
    printf("Updating robot limits from VAL3 server\n");
    Robot* robot = world->robots[0].robot;
    if(!robotInterface.UpdateRobotLimits(*robot))
      return false;

    //choose the collision avoidance margin
    for(size_t i=0;i<robot->geometry.size();i++)
      robot->geometry[i].margin += Real(settings["collision_margin"]);

    robotInterface.GetCurConfig(robot->q);
    robot->UpdateFrames();
    cout<<"Initial configuration: "<<robot->q<<endl;
    cout<<"Initial joint limits: "<<robot->qMin<<endl;
    cout<<"  "<<robot->qMax<<endl;

    plannerSettings.InitializeDefault(*world);
    uis.resize(0);
    //uis.push_back(new IKPlannerCommandInterface);
    uis.push_back(new RRTCommandInterface);
    for(size_t i=0;i<uis.size();i++) {
      uis[i]->world = world;
      uis[i]->robotInterface = &robotInterface;
      uis[i]->viewport = &viewport;
      uis[i]->planningWorld = world;
      uis[i]->settings = &plannerSettings;
      dynamic_cast<InputProcessingInterface*>((RobotUserInterface*)uis[i])->SetProcessor(serialInputProcessor);
      //must be within 0.5 RMSE radians of the start configuration
      dynamic_cast<PlannerCommandInterface*>((RobotUserInterface*)uis[i])->startObjectiveThreshold = 0.5;
    }
    currentUI = oldUI = 0;

    //read from val3 server
    Vector x;
    robotInterface.GetCurConfig(x);
    world->robots[0].robot->UpdateConfig(x);

    //activate current UI
    string res=uis[currentUI]->ActivateEvent(true);

    if(!WorldViewProgram::Initialize()) return false;

    glui = GLUI_Master.create_glui_subwindow(main_window,GLUI_SUBWINDOW_RIGHT);
    glui->set_main_gfx_window(main_window);
    ui_listbox = glui->add_listbox("UI",&currentUI,UI_LISTBOX_ID,ControlFunc);
    for(size_t i=0;i<uis.size();i++) {
      char buf[256];
      strcpy(buf,uis[i]->Description().c_str());
      ui_listbox->add_item(i,buf);
    }

    printf("Done initializing...\n");
    return true;
  }

  virtual void RenderWorld()
  {
    Robot* robot=world->robots[0].robot;

    /*
    PlannerCommandInterface* pInterface = dynamic_cast<PlannerCommandInterface*>(&*uis[currentUI]);
    if(pInterface) {
      robot->UpdateConfig(pInterface->planner->CurrentDestination());
    }
    */
    //draw current predicted configuration from mirror
    const CS8Emulator& mirror=val3.client.Mirror();
    TrajClient::Vector qcur;
    mirror.GetConfig(qcur);
    val3.ConfigFromTrajClient(qcur,robot->q);
    robot->UpdateFrames();
    world->robots[0].view.SetGrey();
    WorldViewProgram::RenderWorld();

    //draw path
      Real tstart = robotInterface.GetCurTime();
      Real tend = robotInterface.GetEndTime();
      Real dt = 0.05;
      //draw end effector path
      glDisable(GL_LIGHTING);
      glColor3f(1,1,0);
      glLineWidth(2.0);
      glBegin(GL_LINES);
      int istart=(int)Ceil(tstart/dt);
      int iend=(int)Ceil(tend/dt);
      for(int i=istart;i<iend;i++) {
	Real t1=i*dt;
	Real t2=t1+0.5*dt;
	robotInterface.GetConfig(t1,robot->q);
	robot->UpdateFrames();
	glVertex3v(robot->links.back().T_World.t);
	robotInterface.GetConfig(t2,robot->q);
	robot->UpdateFrames();
	glVertex3v(robot->links.back().T_World.t);
      }
      glEnd();

    uis[currentUI]->DrawGL();
  }

  virtual void RenderScreen()
  {
    void* fontface = GLUT_BITMAP_TIMES_ROMAN_24;
    const int fontheight = 24;
    const int lineSpacing = 36;
    const int margin = 5;
    int x,y;
    
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    
    x = 20;
    y = 40;

    glColor3f(1,1,1);
    glRasterPos2i(x,y);
    glutBitmapString(fontface,uis[currentUI]->Instructions().c_str());
    y += lineSpacing;

    glEnable(GL_DEPTH_TEST);
  }

  virtual void Handle_Control(int id)
  {
    switch(id) {
    case UI_LISTBOX_ID:
      {
	string res=uis[oldUI]->ActivateEvent(false);
	res=uis[currentUI]->ActivateEvent(true);
	oldUI=currentUI;
      }
      break;
    }
  }
  
  void BeginDrag(int x,int y,int button,int modifiers)
  {
    if(button == GLUT_RIGHT_BUTTON) {
    }
    else {
      WorldViewProgram::BeginDrag(x,y,button,modifiers);
    }
  }

  void EndDrag(int x,int y,int button,int modifiers)
  {
    if(button == GLUT_RIGHT_BUTTON) {
      string res=uis[currentUI]->MouseInputEvent(0,0,true);
    }
  }

  virtual void DoFreeDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragRotate(dx,dy);
    else if(button == GLUT_RIGHT_BUTTON) {
      string res=uis[currentUI]->MouseInputEvent(dx,dy,true);
    }
  }

  virtual void DoCtrlDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragPan(dx,dy);
  }
  
  virtual void DoAltDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragZoom(dx,dy);
  }

  virtual void DoShiftDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON) { camera.dist *= (1 + 0.01*Real(dy)); }
  }

  virtual void Handle_Motion(int x,int y)
  {
    string res=uis[currentUI]->MouseInputEvent(x,y,false);
    Refresh();
  }

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    string res=uis[currentUI]->KeypressEvent(key,x,y);
    Refresh();
  }

  virtual void Handle_Idle() {
    Timer timer;
    val3.AdvanceTime();
    string res=uis[currentUI]->UpdateEvent();

    Refresh();

    SleepIdleCallback(int(Max(0.0,dt-timer.ElapsedTime())*1000.0));
    WorldViewProgram::Handle_Idle();
  }
};


int main(int argc, const char** argv)
{
  if(argc < 2) {
    printf("USAGE: Val3SerialClient XML_file\n");
    return 0;
  }
  RobotWorld world;
  Val3SafeSerialProgram program(&world);
  program.LoadCommandLine(argc,argv);
  return program.Run();
}
