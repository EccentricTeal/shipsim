/*-----------------------------------------------------------------------------
Ship Numerical Model Class


Author: Suisei WADA (D1)
        @Faculty of Naval Architecture and Ocean Engineering, Osaka University
Date:   7th Aug, 2020
-----------------------------------------------------------------------------*/

#ifndef DYNAMIC_SOLVER_HPP
#define DYNAMIC_SOLVER_HPP

//Include Library Headers
#include "mmg_variable.hpp"
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
    ShipPosition3D pos_;
    double dt_;
    std::vector< std::function<auto(auto)> >
};

#endif