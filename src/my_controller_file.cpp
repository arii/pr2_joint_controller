#include "angles/angles.h"
#include "my_controller_pkg/my_controller_file.h"
#include <pluginlib/class_list_macros.h>

using namespace my_controller_ns;
enum Ctrl_Type getControllerType(std::string ctrl_str){
    enum Ctrl_Type ctrl_type;

    if(ctrl_str.compare("position")==0){
      ctrl_type = position;
      ROS_INFO("ctrl type set to position");

    } else if(ctrl_str.compare("velocity")==0) {
      ctrl_type = velocity;
      ROS_INFO("ctrl type set to velocity");

    }else if(ctrl_str.compare("effort")==0){
      ctrl_type = effort;
      ROS_INFO("ctrl type set to effort");

    }else {
      ROS_ERROR("Incorrect controller type given, going to default 'effort'") ;
      ctrl_type = effort;
    }

    return ctrl_type;
}

/// Controller initialization in non-realtime
bool MyControllerClass::init(pr2_mechanism_model::RobotState *robot,
                            ros::NodeHandle &n)
{
  std::string joint_name;
  if (!n.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }

  joint_state_ = robot->getJointState(joint_name);
  if (!joint_state_)
  {
    ROS_ERROR("MyController could not find joint named '%s'",
              joint_name.c_str());
    return false;
  }
  // obtain max_effort 
  double max_effort;
  if(!n.getParam("max_effort",max_effort)){
    ROS_ERROR("No effort given!");
  }else{
    max_effort_ = double(max_effort);
  }
  ROS_INFO("Max effort is : %f", max_effort_);
  
  // obtain initial controller type
  std::string ctrl_str;
  if (!n.getParam("ctrl_type", ctrl_str))
  {
    ROS_ERROR("No controller type given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }else{
    ctrl_type_ = getControllerType(ctrl_str);
  }

  // advertise services
  ctrl_type_srv_ = n.advertiseService("ctrl_type", &MyControllerClass::ctrlType, this); 
  command_srv_ = n.advertiseService("command", &MyControllerClass::command,this);
  is_max_reached_srv_ = n.advertiseService("is_max_reached", 
                                            &MyControllerClass::isMaxReached,this);
  // if position make sure arm doesn't move to 0 
  command_ = joint_state_->position_;
  
  #ifdef PID
  // pid controller initalizations
  robot_ = robot;
  if(!pid_pos_controller_.init(ros::NodeHandle(n, "pid_pos"))){
    ROS_ERROR("cannot find construct pid controller for %s", joint_name.c_str());
    return false;
  }
 if(!pid_vel_controller_.init(ros::NodeHandle(n, "pid_vel"))){
    ROS_ERROR("cannot find construct pid controller for %s", joint_name.c_str());
    return false;
  }
  #endif

  #ifdef DYNAMIC_MAX_EFFORT
  max_effort_srv_ = n.advertiseService("max_effort", &MyControllerClass::setMaxEffort, this);
  #endif
  #ifdef PUBLISH_INFO
  publish_  = n.advertise<my_controller_pkg::ControllerState>("controller_state",5);
  CTRL_STR_[0]= "position";
  CTRL_STR_[1]= "velocity";
  CTRL_STR_[2]= "effort";
  #endif
  #ifdef CAPTURE
    capture_srv_ = n.advertiseService("capture", &MyControllerClass::capture, this);
    mystate_pub_ = n.advertise<my_controller_pkg::MyStateMessage>("mystate_topic",StoreLen);
    storage_index_ = StoreLen;
  #endif
 
  #ifdef FILTER
  memset(vel_buff_,0, NUM_SAMPLES+1);
  memset(eff_buff_,0, NUM_SAMPLES+1);
  v_ptr_ = NUM_SAMPLES;
  e_ptr_ = NUM_SAMPLES;
  #endif
  return true;
}


/// Controller startup in realtime
void MyControllerClass::starting()
{
  init_pos_ = joint_state_->position_;
  #ifdef PID
  time_of_last_cycle_ = robot_->getTime();
  pid_pos_controller_.reset();
  pid_vel_controller_.reset();
  #endif
}


/// Controller update loop in realtime
void MyControllerClass::update()
{
  double current_pos = joint_state_->position_;
  double current_vel = joint_state_->velocity_;
  
  #ifdef PID
  //pid controller
  ros::Duration dt = robot_->getTime() - time_of_last_cycle_;
  time_of_last_cycle_ = robot_->getTime();
  #endif
  double error(0), commanded_effort;
  
  switch(ctrl_type_){ // compute commanded effort for controller type
    case(position):
      // get position error
      switch(joint_state_->joint_->type){
        case(urdf::Joint::REVOLUTE):
          angles::shortest_angular_distance_with_limits(command_, current_pos,joint_state_->joint_->limits->lower, joint_state_->joint_->limits->upper, error);
          break;
        case(urdf::Joint::CONTINUOUS):
          error = angles::shortest_angular_distance(command_, current_pos);
          break;
        default:
          ROS_ERROR("joint was not revolute or continous!");
        break;
      }
      #ifdef PID
      commanded_effort= pid_pos_controller_.updatePid(error, current_vel,dt); 
      #else
      commanded_effort = -10*error;
      #endif
      break;
    case(velocity):
      #ifdef PID
      commanded_effort= pid_vel_controller_.updatePid(current_vel - command_, dt);
      #else 
      commanded_effort= -10*(current_vel - command_);
      #endif
      break;
    case(effort):
      commanded_effort = command_;
      break;
    default:
      commanded_effort = 0;
  }
  bool hit_max = false;

  // some stuff about watching out for too high effot...
  if (commanded_effort > max_effort_){
    commanded_effort = max_effort_;
    hit_max = true;
  }else if (commanded_effort < -1*max_effort_){
    commanded_effort = -1*max_effort_;
    hit_max = true;
  }
  joint_state_->commanded_effort_= commanded_effort;
  max_effort_reached_ = hit_max;
  #ifdef PUBLISH_INFO
  my_controller_pkg::ControllerState msg;
  msg.ctrl_type = CTRL_STR_[ctrl_type_];
  msg.time = ros::Time::now().toSec();
  msg.position = current_pos;
  msg.velocity = current_vel;
  msg.commanded_effort = commanded_effort;
  msg.measured_effort = joint_state_->measured_effort_;
  msg.max_effort_reached = max_effort_reached_;
  #ifdef FILTER
    int i;
    double v_sum = 0, e_sum=0;
    for ( i = 0; i <= NUM_SAMPLES; i++){
      v_sum += vel_buff_[i];
      e_sum += eff_buff_[i];
    }
    msg.filtered_velocity = v_sum;
    msg.filtered_effort = e_sum;

    vel_buff_[v_ptr_] = current_vel;
    eff_buff_[e_ptr_] = commanded_effort;
    v_ptr_ = (v_ptr_==0) ? NUM_SAMPLES : v_ptr_-1;
    e_ptr_ = (e_ptr_==0) ? NUM_SAMPLES : e_ptr_-1;
  
  #else
    msg.filtered_velocity = -1;
    msg.filtered_effort = -1;
  #endif
  publish_.publish(msg);
  #endif
  #ifdef CAPTURE 
    int index = storage_index_;
    if ((index >= 0) && (index < StoreLen))
      {
        storage_[index].time             = ros::Time::now().toSec();
        storage_[index].position         = joint_state_->position_;
        storage_[index].desired_position = desired_pos;
        storage_[index].velocity         = joint_state_->velocity_;
        storage_[index].desired_velocity = 0.0;
        storage_[index].commanded_effort = joint_state_->commanded_effort_;
        storage_[index].measured_effort  = joint_state_->measured_effort_;

        // Increment for the next cycle.
        storage_index_ = index+1;
      }
  #endif
}
/// Controller stopping in realtime
void MyControllerClass::stopping()
{}

/*  Service Functions */

bool MyControllerClass::command(my_controller_pkg::Command::Request& req, 
                                 my_controller_pkg::Command::Response& resp)
{
  command_ = req.command;
  resp.command = command_;
  ROS_INFO("set command = %f", command_);
  return true;
 }

bool MyControllerClass::ctrlType(my_controller_pkg::CtrlTypeSrv::Request& req,
               my_controller_pkg::CtrlTypeSrv::Response& resp)
{
  
  enum Ctrl_Type ctrl_type = getControllerType(req.ctrl_str);
  if(ctrl_type == position){
    // when switching to pos control don't move based on old command
    command_ = joint_state_->position_; 
  }else if(ctrl_type == velocity){
    command_ = 0; // dont move if previous command > 0
  }

  ctrl_type_ = ctrl_type;
  resp.ctrl_str = req.ctrl_str;
  return true;
}

bool MyControllerClass::isMaxReached(my_controller_pkg::IsMaxReached::Request& req,
               my_controller_pkg::IsMaxReached::Response& resp)
{
  ROS_INFO("IsMaxEffortReached service called ");
  resp.is_max_reached = max_effort_reached_;
  return true;
}




#ifdef DYNAMIC_MAX_EFFORT
bool MyControllerClass::setMaxEffort(my_controller_pkg::MaxEffort::Request& req,
                      my_controller_pkg::MaxEffort::Response& resp)
{
  max_effort_ = req.max_effort;
  resp.max_effort = max_effort_;
  ROS_INFO("set max effort = %f ", max_effort_);
  return true;
}
#endif

#ifdef CAPTURE
bool MyControllerClass::capture(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &resp)
{
  ros::Time started = ros::Time::now();
  // mark buffer as clear (which will start storing) 
  storage_index_ = 0;

  //wait until buffer is full
  while(storage_index_ < StoreLen)
  {
    ros::Duration(0.001).sleep();
    if(ros::Time::now() - started > ros::Duration(20.0))
    {
      ROS_ERROR("Buffer never filled up... took longer than 20 seconds");
      return false;
    }
  }

  // publish the contents
  int index;
  for(index = 0; index < StoreLen; index++)
    mystate_pub_.publish(storage_[index]);

  return true;
}
#endif

 // namespace
PLUGINLIB_DECLARE_CLASS(my_controller_pkg,MyControllerPlugin,
                        my_controller_ns::MyControllerClass,
                        pr2_controller_interface::Controller)
