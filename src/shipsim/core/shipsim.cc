/*-----------------------------------------------------------------------------
Author: Suisei WADA (D1)
        @Faculty of Naval Architecture and Ocean Engineering, Osaka University
Date:   8th January, 2021
-----------------------------------------------------------------------------*/
//Include header
#include "shipsim/shipsim.hh"

//ShipsimCore
ShipsimCore::ShipsimCore():
status_( ShipsimStatus::NOT_READY ),
ptr_solver_( new RungeKuttaSolver() ),
simrate_( 10 )
{
	;
}

ShipsimCore::~ShipsimCore()
{
	;
}

ShipsimError ShipsimCore::run( void )
{
	if( status_ == ShipsimStatus::READY )
	{
		return ShipsimError::FAILED_TO_RUN;
	}

	return ShipsimError::OK;
}

ShipsimError ShipsimCore::bindModel( void )
{

	status_ = ShipsimStatus::READY;
	return ShipsimError::OK;
}

