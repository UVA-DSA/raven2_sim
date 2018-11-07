#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <stdlib.h>

/*The purpose of this plugin is to read messages from the Joint_State topic
  , translate their contents into something Gazebo readable, and invoke the gazebo service SetModelConfiguration
  to set the joints to their proper setting.
 */

namespace gazebo
{
class GazeboJointStateTranslator : public ModelPlugin
{
public:

  ros::NodeHandle n;
  ros::Subscriber JointStateSub;
  // ros::Publisher pubDummy; //Test Publisher
  ros::ServiceClient ModelConfigClient;

  GazeboJointStateTranslator() : ModelPlugin()
  {

  }

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    ROS_INFO("Gazebo Joint State Translator successfully loaded.");
    ROS_INFO("GJST: Creating subscriber to /joint_states");

    //Create a subscriber object to /joint_states that reads the messages there.

    JointStateSub =  n.subscribe("joint_states", 1000, &GazeboJointStateTranslator::jointStatesCallback, this);

    if(!JointStateSub){
      ROS_INFO("GJST: Empty subscriber!");
      n.shutdown();

    }
    ROS_INFO("GJST: Subscriber created!");

    //Create the service client that invokes SetModelConfiguration.

    ModelConfigClient = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");




  }

  void Update(physics::ModelPtr _model, sdf::ElementPtr _sdf){


  }

  /*This is where the translation actually happens. When messages are received by the subscriber, they
  trigger the callback function passed as a parameter for the subscriber constructor, with a
  sensor_msgs::Joint_state message passed in as an argument to the callback function.

  The message's contents are reinterpreted (basically, just slapped into a service request) and sent to
  SetModelConfiguration, where Gazebo handles the rest.
  */

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){
    ROS_INFO("Received joint state msg." );

    //Create the service request.
    gazebo_msgs::SetModelConfiguration srv;

    srv.request.model_name = "RavenII";
    srv.request.urdf_param_name = "robot_description";

    //Debug Info: Size of name list in msg, etc.
    ROS_INFO("DEBUG:Size of msg name list : %d", (int)msg->name.size());

    for(int i = 0; i < msg->name.size(); i++){
      srv.request.joint_names.push_back(msg->name[i]);
      srv.request.joint_positions.push_back(msg->position[i]);

      //Debug Info
      ROS_INFO("DEBUG: Added joint name %s to srv: %s", msg->name[i].c_str(), srv.request.joint_names[i].c_str());
      ROS_INFO("DEBUG: Added joint position %f to srv: %f", msg->position[i], srv.request.joint_positions[i]);

    }
    //Call the SetModelConfiguration service.

    if(ModelConfigClient.call(srv)){
      ROS_INFO("DEBUG: Success : %d", srv.response.success);

    }

  }


};

GZ_REGISTER_MODEL_PLUGIN(GazeboJointStateTranslator)

}
