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
simrate_( 10 )
{
  ptr_model_ = std::make_unique< shipsim_model::ShallowWater2019::Model<shipsim::Core> >( this );
  shptr_propctrl_sub_ = this -> create_subscription<shipsim_msg::msg::ShipsimPropCtrl>(
    topicname_propctrl_sub_,
    rclcpp::QoS(1),
    std::bind( &callbackPropCtrl_sub_, this, std::placeholders::_1 )
  );
}

shipsim::Core::~Core()
{
	;
}

shipsim::common::Error shipsim::Core::run( void )
{


}

void shipsim::Core::simulatorMain_( void )
{
  //Local variables
  shipsim_model::ShallowWater2019::DynamicParam dparam;

  //Refresh command to ship
  {
    std::lock_guard<std::mutex> lock(mtx_);
    dparam.propRot = ctrlcmd_mainprop_.rpm;
    dparam.rudAng = ctrlcmd_ctrrud_.angle;
  }

}

void shipsim::Core::callbackPropCtrl_sub_( const shipsim_msg::msg::ShipsimPropCtrl::SharedPtr msg )
{
  std::lock_guard<std::mutex> lock( mtx_ );
  ctrlcmd_mainprop_ = *msg;
}

void shipsim::Core::callbackRudderCtrl_sub_( const shipsim_msg::msg::ShipsimRudderCtrl::SharedPtr msg )
{
  std::lock_guard<std::mutex> lock( mtx_ );
  ctrlcmd_ctrrud_ = *msg;
}