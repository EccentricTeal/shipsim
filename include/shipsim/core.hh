/*-----------------------------------------------------------------------------
Shipsim Core


Author: Suisei WADA (D1)
        @Faculty of Naval Architecture and Ocean Engineering, Osaka University
Date:   7th Feb, 2021
-----------------------------------------------------------------------------*/
#ifndef SHIPSIM_CORE_HH
#define SHIPSIM_CORE_HH

//ROS
#include <rclcpp/rclcpp.hpp>
//ROS Message
#include "shipsim_msgs/msg/propeller_state.hpp"
#include "shipsim_msgs/msg/rudder_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
//TF
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
//Shipsim
#include "shipsim/common/codedef.hh"
//Shipsim Model
#include "shipsim_model/ShallowWater2019/model.hh"
#include "shipsim_model/ShallowWater2019/param.hh"
#include "shipsim_model/common/shipstatedef.hh"
//STL
#include <memory>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>

namespace shipsim
{
  class Core : public rclcpp::Node
  {
    /* Constractor, Destructor */
    public:
      Core( std::string node_name, std::string name_space );
      ~Core();

    /* Public method, Accessor */
    public:
      void run( void );
      void stop( void );
    
    /* Private variables */
    private:
      //ROS
      std::unique_ptr<tf2_ros::TransformBroadcaster> unqptr_dynamictf_shippos_br_;
      rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr shptr_shipvel_pub_;
      rclcpp::Subscription<shipsim_msgs::msg::PropellerState>::SharedPtr shptr_prop_sub_;
      rclcpp::Subscription<shipsim_msgs::msg::RudderState>::SharedPtr shptr_rudder_sub_;

      //Simulator
      shipsim::common::Status simstate_;
      std::chrono::nanoseconds simrate_;
      std::chrono::nanoseconds pubrate_, subrate_;

      //Model
      std::unique_ptr<shipsim_model::ShallowWater2019::Model<shipsim::Core>> ptr_model_;
      shipsim_model::common::ShipStateParam shipstate_;
      shipsim_model::ShallowWater2019::DynamicParam ctrlparam_;

      //Parameter
      std::string topicname_shipvel_pub_;
      std::string topicname_propctrl_sub_;
      std::string topicname_rudctrl_sub_;
      std::string frameid_world_;
      std::string frameid_ship_;

      //Data
      shipsim_msgs::msg::PropellerState ctrlcmd_mainprop_;
      shipsim_msgs::msg::RudderState ctrlcmd_ctrrud_;
      geometry_msgs::msg::TransformStamped shippos_;
      geometry_msgs::msg::TwistStamped shipvel_;

      //Thread
      std::unique_ptr<std::thread> unqptr_thread_sim_;
      std::unique_ptr<std::thread> unqptr_thread_topicpub_;
      std::unique_ptr<std::thread> unqptr_thread_topicsub_;
    
      //Functional
      std::mutex mtx_;
      bool enableRun_;

    /* Private Merhods, functions */
    private:
      //Main Loop
      void simulate_( void );
      void subscribe_( void );
      void publish_( void );

      //Subscriber Call Back
      void callbackPropCtrl_sub_( const shipsim_msgs::msg::PropellerState::SharedPtr msg );
      void callbackRudderCtrl_sub_( const shipsim_msgs::msg::RudderState::SharedPtr msg );
  };

}




#endif