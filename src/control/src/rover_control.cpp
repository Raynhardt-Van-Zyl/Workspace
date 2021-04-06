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
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

#include "std_msgs/String.h"

#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>

namespace gazebo
{
  class Control : public ModelPlugin
  {
    // \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    private: ros::NodeHandle publisherNode;

    // \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    // A ROS publisher
    private: ros::Publisher telemetries;

    // \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    // \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    private: float left[20];

    private: float tleft[20];

    private: float right[20];

    private: float tright[20];

    private: double time;

    private: bool received;



    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {

      this->received = false;

      this->time = 0;

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
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
            "/" + this->model->GetName() + "/set_control_Input",
            1,
            boost::bind(&Control::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);
      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&Control::QueueThread, this));


    }

    // \brief Handle an incoming message from ROS
    // \param[in] _msg A float value that is used to set the velocity
    // of the Velodyne.
    public: void OnRosMsg(const std_msgs::Float32MultiArrayConstPtr &msg)
    {
      this->received = true;
      int count = 0;
      for (size_t i = 0; i < 20; i++)
      {
        /* code */
        this->tleft[i] = msg->data[count];
        this->left[i] = msg->data[count + 1];
        this->tright[i] = msg->data[count + 2];
        this->right[i] = msg->data[count + 3];
        this->time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        count += 4;
      }

      
    }

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    private: void controlUpdate(){

      if (this->received)
      {
        double timeStep = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - this->time;
        int temp = 0;
        while (this->tleft[temp]*1000 < timeStep){
          temp++;
          if (temp >=39)
          {
            break;
          }
          
        }
        geometry_msgs::Pose lidar = geometry_msgs::Pose();

        // this->model->GetJoint("lidar_joint")

        // this->model->GetLink("lidar")->SetRelativePose((ignition::math::v6::Vector3(0.0, (double)right[temp],0.0)))
        // ROS_INFO("right: [%lf]", (double)right[temp]);
        // ROS_INFO("left: [%lf]", (double)left[temp]);
        // ROS_INFO("left: [%d]", temp);
        // this->model->GetLink("right_front_wheel")->SetAngularVel(ignition::math::v6::Vector3(0.0, (double)right[temp],0.0));
        // this->model->GetLink("right_back_wheel")->SetAngularVel(ignition::math::v6::Vector3(0.0,(double)right[temp],0.0));
        // this->model->GetLink("left_front_wheel")->SetAngularVel(ignition::math::v6::Vector3(0.0,(double)left[temp],0.0));
        // this->model->GetLink("left_back_wheel")->SetAngularVel(ignition::math::v6::Vector3(0.0,(double)left[temp],0.0));
      }
      

      
      


      

    }



    // Called by the world update start event
    public: void OnUpdate()
    {

      controlUpdate();

    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Control)
}
