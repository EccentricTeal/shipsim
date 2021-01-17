/*-----------------------------------------------------------------------------
Ship Numerical Model Class


Author: Suisei WADA (D1)
        @Faculty of Naval Architecture and Ocean Engineering, Osaka University
Date:   17th Jan, 2021
-----------------------------------------------------------------------------*/

#ifndef SHIPSIM_CORE_SOLVER_HH
#define SHIPSIM_CORE_SOLVER_HH

/* Include Headers */
//ROS
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
//STL
#include <functional>
#include <memory>
#include <array>
#include <exception>
//Simulator Specific
#include "shipsim_model/common/shipstatedef.hh"


//  1: Solver with Runge-Kutta Method for MMG Simulator:
namespace shipsim::core::solver
{
    

    class RungeKuttaSolver
    {
        /* Constractor and Destructor */
        public:
        RungeKuttaSolver();
        ~RungeKuttaSolver();

        /* Public methods (Accessor) */
        public:
        void registGradUpdateFunc( std::function<geometry_msgs::msg::Twist(void)>& func );
        void registStateSetFunc( std::function<void(shipsim_model::common::ShipStateParam)>& func );
        shipsim_model::common::ShipStateParam solve( shipsim_model::common::ShipStateParam state, double dt );

        /* Private methods */
        geometry_msgs::msg::Twist calcAcc_( void );
        geometry_msgs::msg::Twist calcVel_( double dt );
        geometry_msgs::msg::Twist calcPos_( double dt );
        

        // Member variables
        private:
        shipsim_model::common::ShipStateParam nowData_;
        shipsim_model::common::ShipStateParam nextData_;
        std::function<geometry_msgs::msg::Twist(void)> updateGrad_;
        std::function<void(shipsim_model::common::ShipStateParam)> setState_;
    };

}


#endif