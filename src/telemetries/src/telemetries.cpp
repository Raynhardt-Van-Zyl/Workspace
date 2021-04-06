#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include "geometry_msgs/Pose.h"

#include "std_msgs/String.h"

#include <tf2_ros/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include <nav_msgs/Odometry.h>

namespace gazebo
{
  class Control : public ModelPlugin
  {
    // \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;


    // A ROS publisher
    private: ros::Publisher telemetries;

    // broadcast tf of chassis
    private: tf2_ros::TransformBroadcaster chassis_broadcaster;

    // broadcast tf of lidar
    private: tf2_ros::TransformBroadcaster lidar_broadcaster;

    // \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    // \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {

      // Make sure the ROS node for Gazebo has already been initialized

      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&Control::OnUpdate, this));
      
      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "telemetries",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("telemetries"));


      this->telemetries = this->rosNode->advertise<nav_msgs::Odometry>("odom", 50); 

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&Control::QueueThread, this));


    }

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }


    // Called by the world update start event
    public: void OnUpdate()
    {

      ros::Time current_time;
      current_time = ros::Time::now();

      ignition::math::v6::Pose3d temp =  this->model->GetLink("chassis")->WorldPose();
      ignition::math::v6::Pose3d temp2 =  this->model->GetLink("lidar")->RelativePose();

      //since all odometry is 6DOF we'll need a quaternion created from yaw
      geometry_msgs::Quaternion chassis_quat = tf::createQuaternionMsgFromRollPitchYaw(temp.Roll(),temp.Pitch(),temp.Yaw());
      geometry_msgs::Quaternion lidar_quat = tf::createQuaternionMsgFromRollPitchYaw(temp2.Roll(),temp2.Pitch(),temp2.Yaw());
      auto temp3 = this->model->GetLink("chassis")->WorldLinearVel();

      //map to world
      geometry_msgs::TransformStamped world;
      world.header.stamp = ros::Time::now();
      world.header.frame_id = "map";
      world.child_frame_id = "world";

      world.transform.translation.x = 0.0;
      world.transform.translation.y = 0.0;
      world.transform.translation.z = 0.0;
      world.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);

      //send the transform
      chassis_broadcaster.sendTransform(world);

      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped world_chassis;
      world_chassis.header.stamp = ros::Time::now();
      world_chassis.header.frame_id = "world";
      world_chassis.child_frame_id = "chassis";

      world_chassis.transform.translation.x = temp.X();
      world_chassis.transform.translation.y = temp.Y();
      world_chassis.transform.translation.z = temp.Z();
      world_chassis.transform.rotation = chassis_quat;

      //send the transform
      chassis_broadcaster.sendTransform(world_chassis);

      

      geometry_msgs::TransformStamped chassis_lidar;
      chassis_lidar.header.stamp = ros::Time::now();
      chassis_lidar.header.frame_id = "chassis";
      chassis_lidar.child_frame_id = "lidar";

      chassis_lidar.transform.translation.x = temp2.X();
      chassis_lidar.transform.translation.y = temp2.Y();
      chassis_lidar.transform.translation.z = temp2.Z();
      chassis_lidar.transform.rotation = lidar_quat;

      //send the transform
      lidar_broadcaster.sendTransform(chassis_lidar);

      

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time::now();
      odom.header.frame_id = "chassis";
      odom.child_frame_id = "lidar";

      //set the position
      odom.pose.pose.position.x = temp2.X();
      odom.pose.pose.position.y = temp2.Y();
      odom.pose.pose.position.z = temp2.Z();
      odom.pose.pose.orientation = lidar_quat;


      //set the velocity
      odom.twist.twist.linear.x = temp3.X();
      odom.twist.twist.linear.y = temp3.Y();
      odom.twist.twist.angular.z = temp3.Z();

      //publish the message
      telemetries.publish(odom);

    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Control)
}
