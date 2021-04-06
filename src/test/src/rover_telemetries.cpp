#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <std_msgs/String.h>

namespace gazebo
{
  ignition::math::v6::Pose3d pose;
  class RoverTelemetries : public ModelPlugin
  {

  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("rover_telemetries", 1000);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&RoverTelemetries::OnUpdate, this));
    }

    // Called by the world update start event
  public:
    void OnUpdate()
    {
      // Get link state
      pose = this->model->DirtyPose();
    }

    // Pointer to the model
  private:
    physics::ModelPtr model;

    // Pointer to the update event connection
  private:
    event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RoverTelemetries)

  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "rover_telemetries");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("rover_telemetries", 1000);

    RoverTelemetries tel;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
      /**
     * This is a message object. You stuff it with data, and then publish it.
     */
      std_msgs::String msg;

      std::stringstream ss;
      ss << "Position: " << pose.Pos().X() << pose.Pos().Y() << pose.Pos().Z();
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());

      /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
      chatter_pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
    }
    return 0;
  }
}
