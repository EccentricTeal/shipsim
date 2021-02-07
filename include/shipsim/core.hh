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

namespace shipsim
{
    class Core
    {
        /* Constractor, Destructor */
        public:
            Core( rclcpp::Node& node );
            ~Core();

        /* Public method, Accessor */
        public:
            shipsim::common::Error run( void );
        
        /* Private variables */
        private:
            //Simulator
            shipsim::common::Status simstate_;
            unsigned int simrate_; //Hz

            //Model
            std::unique_ptr<shipsim_model::ShallowWater2019::Model> ptr_model_;
            shipsim_model::common::ShipStateParam shipstate_;
            shipsim_model::ShallowWater2019::DynamicParam ctrlparam_;

    };

}




#endif