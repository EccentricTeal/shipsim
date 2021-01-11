/*-----------------------------------------------------------------------------
Ship Numerical Model Class


Author: Suisei WADA (D1)
        @Faculty of Naval Architecture and Ocean Engineering, Osaka University
Date:   7th Aug, 2020
-----------------------------------------------------------------------------*/

#ifndef SHIPSIM_DYNAMIC_SOLVER_HH
#define SHIPSIM_DYNAMIC_SOLVER_HH

//Include Library Headers
#include "mmg_variable.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <functional>
#include <vector>

//Include Tools


//  1: Solver with Runge-Kutta Method for MMG Simulator:
class RungeKuttaSolver
{
    // Constractor and Destructor
    public:
    RungeKuttaSolver();
    ~RungeKuttaSolver();

    // Public method, Accessor
    public:
    void addFunction();
    void setInit( void );
    void calcNextStep( void );
    void getPosition( void );

    // Member variables
    private:
    geometry_msgs::msg::TwistStamped pos_;
    geometry_msgs::msg::TwistStamped vel_;
    geometry_msgs::msg::TwistStamped acc_;
    double dt_;
    std::vector< std::function<auto(auto)> a;
};

#endif