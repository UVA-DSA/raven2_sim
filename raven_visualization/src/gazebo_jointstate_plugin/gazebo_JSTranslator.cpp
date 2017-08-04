#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

/*The purpose of this plugin is to read messages from the Joint_State topic 
  , translate their contents into something Gazebo readable, and invoke the gazebo service 
 */

namespace gazebo
{
class GazeboJointStateTranslator : public ModelPlugin
{
public:
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

    ROS_INFO("Gazebo Joint State Translator successfully loaded. \n");
  }

};
GZ_REGISTER_MODEL_PLUGIN(GazeboJointStateTranslator)
}
