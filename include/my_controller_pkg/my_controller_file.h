#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <ros/ros.h>
#include <my_controller_pkg/Command.h>
#include <my_controller_pkg/CtrlTypeSrv.h>
#include <my_controller_pkg/MaxCurrentReached.h>
#include <my_controller_pkg/IsMaxReached.h>
#include <my_controller_pkg/MaxEffort.h>
#include <std_srvs/Empty.h>
#include "my_controller_pkg/MyStateMessage.h"
#include "my_controller_pkg/ControllerState.h"
#include <control_toolbox/pid.h>

/*  improve service performance by commenting these debug services  */
//#define CAPTURE
#define PID 
#define DYNAMIC_MAX_EFFORT // uncomment to stop allowing service calls to change max effort
#define PUBLISH_INFO  
#define FILTER  // using moving filter for velocity and effort measurements

namespace my_controller_ns{
#ifdef CAPTURE
enum
{
  StoreLen = 5000 // 5000 samples -> 5 second time window
};
#endif

enum Ctrl_Type {
  position,
  velocity,
  effort
};

#ifdef FILTER
const int NUM_SAMPLES = 9;
#endif

class MyControllerClass: public pr2_controller_interface::Controller
{
private:
  
  /*  controller variables  */
  pr2_mechanism_model::JointState* joint_state_;

  double init_pos_;
  double command_;     
  double amplitude_;
  double max_effort_;
  bool max_effort_reached_;
  enum Ctrl_Type ctrl_type_;
  ros::ServiceServer ctrl_type_srv_;  // switch controller type service
  ros::ServiceServer command_srv_;    // command  service ( similiar to command msg )
  ros::ServiceServer is_max_reached_srv_; 
  /*  service functions  */
  bool command(my_controller_pkg::Command::Request& req,
               my_controller_pkg::Command::Response& resp);
  bool ctrlType(my_controller_pkg::CtrlTypeSrv::Request& req,
               my_controller_pkg::CtrlTypeSrv::Response& resp);
  bool isMaxReached(my_controller_pkg::IsMaxReached::Request& req,
               my_controller_pkg::IsMaxReached::Response& resp);
  #ifdef PID 
  // pid controller variables
  control_toolbox::Pid pid_pos_controller_;
  control_toolbox::Pid pid_vel_controller_;
  pr2_mechanism_model::RobotState* robot_;
  ros::Time time_of_last_cycle_;
  #endif
  
  #ifdef PUBLISH_INFO
  ros::Publisher publish_;
  const char* CTRL_STR_[3];
  #endif 

  #ifdef DYNAMIC_MAX_EFFORT
  ros::ServiceServer max_effort_srv_;
  bool setMaxEffort(my_controller_pkg::MaxEffort::Request& req,
                      my_controller_pkg::MaxEffort::Response& resp);
  #endif

  #ifdef CAPTURE
    bool capture(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
    ros::ServiceServer capture_srv_;
    ros::Publisher mystate_pub_;
    my_controller_pkg::MyStateMessage storage_[StoreLen];
    volatile int storage_index_;
  #endif 
    
  #ifdef FILTER
    double vel_buff_ [NUM_SAMPLES +1 ];
    int v_ptr_, e_ptr_;
    double eff_buff_[NUM_SAMPLES +1];
    int eff_head_, eff_tail_;

  #endif
public:
  virtual bool init(pr2_mechanism_model::RobotState *robot,
                   ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();
};
} 
