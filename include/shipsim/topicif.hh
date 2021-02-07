/*-----------------------------------------------------------------------------
Shipsim Topic Interface


Author: Suisei WADA (D1)
        @Faculty of Naval Architecture and Ocean Engineering, Osaka University
Date:   8th Feb, 2021
-----------------------------------------------------------------------------*/
#ifndef SHIPSIM_TOPICIF_HH
#define SHIPSIM_TOPICIF_HH

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

namespace shipsim
{
    class TopicInterface
    {
        /* Constractor, Destructor */
        public:
            TopicInterface( rclcpp::Node& node );
            ~TopicInterface();

        /* Public method, Accessor */
        public:
            shipsim::common::Error run( void );
        
        /* Private variables */
        private:
            //ROS Objects
            std::shared_ptr<SubscriptionT>

    };

}




#endif