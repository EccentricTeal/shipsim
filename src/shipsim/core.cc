/*-----------------------------------------------------------------------------
Author: Suisei WADA (D1)
        @Faculty of Naval Architecture and Ocean Engineering, Osaka University
Date:   7th February, 2021
-----------------------------------------------------------------------------*/
//Include header
#include "shipsim/core.hh"

//ShipsimCore
shipsim::Core::Core( rclcpp::Node& node ):
ptr_model_( new shipsim_model::ShallowWater2019::Model( node )  ),
simrate_( 10 )
{
	
}

shipsim::Core::~Core()
{
	;
}

shipsim::common::Error shipsim::Core::run( void )
{

}


