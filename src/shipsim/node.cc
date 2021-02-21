#include "shipsim/core.hh"


int main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  shipsim::Core shipsim_node( "ShipSimNode", "/shipsim/" );

  shipsim_node.run();

  rclcpp::shutdown();
}