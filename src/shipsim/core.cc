/*-----------------------------------------------------------------------------
Author: Suisei WADA (D1)
        @Faculty of Naval Architecture and Ocean Engineering, Osaka University
Date:   7th February, 2021
-----------------------------------------------------------------------------*/
//Include header
#include "shipsim/core.hh"

//ShipsimCore
shipsim::Core::Core( std::string node_name, std::string name_space ):
rclcpp::Node( node_name, name_space ),
shptr_shipvel_pub_( nullptr ),
shptr_prop_sub_( nullptr ),
shptr_rudder_sub_( nullptr )
{
  using namespace std::literals::chrono_literals;

  //Simulator Param
  simrate_ = 100ms;
  pubrate_ = 50ms;
  subrate_ = 50ms;

  //Model
  unqptr_model_ = std::make_unique<shipsim_model::ShallowWater2019::Model>( *this );


  //Publisher
  shptr_shipvel_pub_ = this -> create_publisher<geometry_msgs::msg::TwistStamped>(
    topicname_shipvel_pub_,
    rclcpp::QoS(1)
  );

  //Subscriber
  shptr_prop_sub_ = this -> create_subscription<shipsim_msgs::msg::PropellerState>(
    topicname_propctrl_sub_,
    rclcpp::QoS(1),
    std::bind( &shipsim::Core::callbackPropCtrl_sub_, this, std::placeholders::_1 )
  );
  shptr_rudder_sub_ = this -> create_subscription<shipsim_msgs::msg::RudderState>(
    topicname_rudctrl_sub_,
    rclcpp::QoS(1),
    std::bind( &shipsim::Core::callbackRudderCtrl_sub_, this, std::placeholders::_1 )
  );

  //TF
  unqptr_dynamictf_shippos_br_ = std::make_unique<tf2_ros::TransformBroadcaster>( *this );

  //Initalizing member variables
  shippos_.header.frame_id = frameid_world_;
  shippos_.child_frame_id = frameid_ship_;
}

shipsim::Core::~Core()
{
	stop();
}

void shipsim::Core::run( void )
{
  enableRun_ = true;
  unqptr_thread_sim_ = std::make_unique<std::thread>( std::bind( &shipsim::Core::simulate_, this ) );
  unqptr_thread_topicpub_ = std::make_unique<std::thread>( std::bind( &shipsim::Core::publish_, this ) );
  unqptr_thread_topicsub_ = std::make_unique<std::thread>( std::bind( &shipsim::Core::subscribe_, this ) );
}

void shipsim::Core::stop( void )
{
  enableRun_ = false;
  if( unqptr_thread_sim_ -> joinable() )
  {
    unqptr_thread_sim_ -> join();
  }

  if( unqptr_thread_topicpub_ -> joinable() )
  {
    unqptr_thread_topicpub_ -> join();
  }

  if( unqptr_thread_topicsub_ -> joinable() )
  {
    unqptr_thread_topicsub_ -> join();
  }
  
}

void shipsim::Core::simulate_( void )
{
  //Local variables
  shipsim_model::ShallowWater2019::DynamicParam dparam;
  shipsim_model::common::ShipStateParam sparam;
  rclcpp::Clock simClock( RCL_ROS_TIME );
  rclcpp::Time time_prev = simClock.now();
  rclcpp::Time time_now = simClock.now();
  rclcpp::Duration dt( time_now.seconds(), time_now.nanoseconds() );
  tf2::Quaternion angularQ;
  rclcpp::WallRate loop_rate( simrate_ );

  while ( rclcpp::ok() && enableRun_ )
  {
    //Get time
    time_now = simClock.now();
    dt = time_now - time_prev;

    //Refresh command to ship
    {
      std::lock_guard<std::mutex> lock( mtx_ );
      dparam.propRot = ctrlcmd_mainprop_.rpm;
      dparam.rudAng = ctrlcmd_ctrrud_.angle;
      dparam.rudVel = ctrlcmd_ctrrud_.vel;
    }
    unqptr_model_->setDynamicParam( dparam );

    //Update
    unqptr_model_->update( dt.seconds() );

    //Store new shipstate
    sparam = unqptr_model_->getShipState();
    angularQ.setRPY( sparam.pos.angular.x, sparam.pos.angular.y, sparam.pos.angular.z );
    {
      std::lock_guard<std::mutex> lock( mtx_ );
      shippos_.transform.translation = sparam.pos.linear;
      shippos_.transform.rotation.x = angularQ.x();
      shippos_.transform.rotation.y = angularQ.y();
      shippos_.transform.rotation.z = angularQ.z();
      shippos_.transform.rotation.w = angularQ.w();
      shippos_.header.stamp = time_now;      
    }

    time_prev = time_now;
    loop_rate.sleep();
  }
  
}

void shipsim::Core::publish_( void )
{
  geometry_msgs::msg::TransformStamped shippos;
  geometry_msgs::msg::TwistStamped shipvel;
  rclcpp::WallRate loop_rate( pubrate_ );

  while( rclcpp::ok() && enableRun_ )
  {
    //Get Data
    {
      std::lock_guard<std::mutex> lock( mtx_ );
      shippos = shippos_;
      shipvel = shipvel_;
    }

    unqptr_dynamictf_shippos_br_->sendTransform( shippos );
    shptr_shipvel_pub_ -> publish( shipvel );
    loop_rate.sleep();
  }
}

void shipsim::Core::subscribe_( void )
{
  rclcpp::WallRate loop_rate( subrate_ );

  while( rclcpp::ok() && enableRun_ )
  {
    rclcpp::spin_some( this->get_node_base_interface() );
    loop_rate.sleep();
  }
}

void shipsim::Core::callbackPropCtrl_sub_( const shipsim_msgs::msg::PropellerState::SharedPtr msg )
{
  std::lock_guard<std::mutex> lock( mtx_ );
  ctrlcmd_mainprop_ = *msg;
}

void shipsim::Core::callbackRudderCtrl_sub_( const shipsim_msgs::msg::RudderState::SharedPtr msg )
{
  std::lock_guard<std::mutex> lock( mtx_ );
  ctrlcmd_ctrrud_ = *msg;
}