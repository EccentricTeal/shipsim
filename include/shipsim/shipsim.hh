#ifndef SHIPSIM_SHIPSIM_HH
#define SHIPSIM_SHIPSIM_HH

//ROS
#include <rclcpp/rclcpp.hpp>
//Shipsim
#include "shipsim/error.hh"
#include "shipsim/dynamic_solver.hh"

//STL
#include <memory>


class ShipsimCore
{
    /* Constractor, Destructor */
    public:
    ShipsimCore();
    ~ShipsimCore();

    /* Public method, Accessor */
    public:
    ShipsimError run( void );
    ShipsimError bindModel( void );
    
    /* Private variables */
    private:
    ShipsimStatus status_;
    std::unique_ptr<RungeKuttaSolver> ptr_solver_;
    unsigned int simrate_; //Hz

};







#endif