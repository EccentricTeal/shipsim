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
//Shipsim
#include "shipsim/common/codedef.hh"
//Shipsim Message
#include "shipsim_msg/msg/shipsim_prop_ctrl.hpp"
#include "shipsim_msg/msg/shipsim_rudder_ctrl.hpp"
//Shipsim Model
#include "shipsim_model/ShallowWater2019/model.hh"
#include "shipsim_model/ShallowWater2019/param.hh"
#include "shipsim_model/common/shipstatedef.hh"
//STL
#include <memory>
#include <string>
#include <mutex>
#include <thread>

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
      shipsim::common::Error run( void );
    
    /* Private variables */
    private:
      //ROS
      rclcpp::Subscription<shipsim_msg::msg::ShipsimPropCtrl>::SharedPtr shptr_propctrl_sub_;

      //Simulator
      shipsim::common::Status simstate_;
      unsigned int simrate_; //Hz

      //Model
      std::unique_ptr<shipsim_model::ShallowWater2019::Model<shipsim::Core>> ptr_model_;
      shipsim_model::common::ShipStateParam shipstate_;
      shipsim_model::ShallowWater2019::DynamicParam ctrlparam_;

      //Parameter
      std::string topicname_propctrl_sub_;

      //Data
      shipsim_msg::msg::ShipsimPropCtrl ctrlcmd_mainprop_;
      shipsim_msg::msg::ShipsimRudderCtrl ctrlcmd_ctrrud_;

      //Functional
      std::mutex mtx_;

    /* Private Merhods, functions */
    private:
      //Main Loop
      void simulatorMain_( void );

      //Subscriber Call Back
      void callbackPropCtrl_sub_( const shipsim_msg::msg::ShipsimPropCtrl::SharedPtr msg );
      void callbackRudderCtrl_sub_( const shipsim_msg::msg::ShipsimRudderCtrl::SharedPtr msg );
  };

}




#endif