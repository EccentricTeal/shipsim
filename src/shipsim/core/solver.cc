#include "shipsim/core/solver.hh"


shipsim::core::solver::RungeKuttaSolver::RungeKuttaSolver():
updateGrad_(nullptr),
setState_(nullptr)
{
    ;
}

shipsim::core::solver::RungeKuttaSolver::~RungeKuttaSolver()
{
    ;
}

void shipsim::core::solver::RungeKuttaSolver::registGradUpdateFunc( std::function<geometry_msgs::msg::Twist(void)>& func )
{
    updateGrad_ = func;
}

void shipsim::core::solver::RungeKuttaSolver::registStateSetFunc( std::function<void(shipsim_model::common::ShipStateParam)>& func )
{
    setState_ = func;
}

shipsim_model::common::ShipStateParam shipsim::core::solver::RungeKuttaSolver::solve( shipsim_model::common::ShipStateParam state, double dt )
{
    //Check whether two functions have already been registered
    if( updateGrad_ == nullptr || setState_ == nullptr )
    {
        printf("[ERROR] Not registered model to Runge-Kutta Solver!");
        std::terminate();
    }

    //Declaration
    nowData_ = state;

    //Solve
    nextData_.acc = calcAcc_();
    nextData_.vel = calcVel_( dt );
    nextData_.pos = calcPos_( dt );

    return nextData_;
}

geometry_msgs::msg::Twist shipsim::core::solver::RungeKuttaSolver::calcAcc_( void )
{
    setState_( nowData_ );
    geometry_msgs::msg::Twist acc = updateGrad_();

    return acc;
}

geometry_msgs::msg::Twist shipsim::core::solver::RungeKuttaSolver::calcVel_( double dt )
{

    //Declaration
    geometry_msgs::msg::Twist acc;
    std::array<double, 6> accAr;
    shipsim_model::common::ShipStateParam tempState = nowData_;
    std::array< std::array<double,6>, 4 > K;
    std::array<double, 6> nowvel =
    {
        nowData_.vel.linear.x,
        nowData_.vel.linear.y,
        nowData_.vel.linear.z,
        nowData_.vel.angular.x,
        nowData_.vel.angular.y,
        nowData_.vel.angular.z
    };

    //Calc k1    
    setState_( nowData_ );
    acc = updateGrad_();
    accAr ={ acc.linear.x, acc.linear.y, acc.linear.z, acc.angular.x, acc.angular.y, acc.angular.z };
    for(int i=0; i<6; i++){ K[0][i] = accAr[i] * dt; }

    //Calc k2
    tempState = nowData_;
    tempState.vel.linear.x = nowvel[0] + ( 0.5 * K[0][0] );
    setState_( tempState )
    acc = updateGrad_();
    K[1][0] += acc.linear.x * dt;

    tempState = nowData_;
    tempState.vel.linear.y = nowvel[1] + ( 0.5 * K[0][1] );
    setState_( tempState )
    acc = updateGrad_();
    K[1][0] += acc.linear.x * dt;

    tempState = nowData_;
    tempState.vel.linear.z = nowvel[2] + ( 0.5 * K[0][2] );
    setState_( tempState )
    acc = updateGrad_();
    K[1][0] += acc.linear.x * dt;
}