/*-----------------------------------------------------------------------------
Ship Numerical Model Class


Author: Suisei WADA (D1)
        @Faculty of Naval Architecture and Ocean Engineering, Osaka University
Date:   10th May, 2020
-----------------------------------------------------------------------------*/

#ifndef MMG_MODELS_HPP
#define MMG_MODELS_HPP

//Include Library Headers
#include "physical_const.hpp"
#include "mmg_parameter.hpp"
#include "mmg_variable.hpp"
#include <Eigen/Core>

//Include Tools
#include <vector>
#include <cmath>


//  1:MMG Model on shallow water:
//　Not Thread safe
//
//  [Base Article Information]
//  (Title:   "Maneuvering Simulations of a Ship in Shallow Water)
//  (Authors: Takanori SUZUKI, Hironori YASUKAWA et al. of Hiroshima Univ.)
//  (Journal: JASNAOE Spring Conference 2019 Proceedings Vol.28 pp.491-496)
class ShallowWaterModel
{
	
	public:
	//Initialize and Finalize Functions
	ShallowWaterModel();	//Constractor
	~ShallowWaterModel();  //Destructor

	//Accessor
	void update(void);
	double getForceX(void);
	double getForceY(void);
	double getForceN(void);
	double getForceK(void);

	private:
	//Physical Constants
	PhysicalConst phyconst_;
	
	//Ship Control Variables
	std::vector<PropellerStatus> propeller_;
	std::vector<RudderStatus> rudder_;

	//Ship Dynamics Variables
	ShipPosition3D pos_;
	ShipVelocity3D vel_;
	ShipAcceralator3D acc_;
	
	
	//Ship Parameters
	BodyParam bodyparam_;
	MassParam massparam_;
	PropellerParam propellerparam_;
	RudderParam rudderparam_;
	MmgParam mmgparam_;

	//Force and Moment
	FlagParam X_;
	FlagParam Y_;
	FlagParam N_;	
	FlagParam K_;

	//Temporary Variables
	//-- first: isUpdated flag,  second: value--
	FlagParam tempU_;
	FlagParam tempF_N_;
	FlagParam tempX_H_;
	FlagParam tempY_H_;
	FlagParam tempN_H_;
	FlagParam tempX_P_;
	FlagParam tempX_R_;
	FlagParam tempY_R_;
	FlagParam tempN_R_;
	
	//Local Calculator
	void calcU_(void); //Calculate U temporarily
	void calcF_N_(void); //Calculate F_N temporarily
	
	void calcX_H_(void); //Calculate Surge force correspondings to Hull
	void calcY_H_(void); //Calculate Sway force correspondings to Hull
	void calcN_H_(void); //Calculate Yaw moment correspondings to Hull
	void calcX_P_(void); //Calculate Surge force correspondings to Propeller
	void calcX_R_(void); //Calculate Surge force correspondings to Rudder
	void calcY_R_(void); //Calculate Sway force correspondings to Rudder
	void calcN_R_(void); //Calculate Yaw moment correspondings to Rudder

	void updateX_(void); //Update Surge Force
	void updateY_(void); //Update Sway Force
	void updateN_(void); //Update Yaw Moment
	void updateK_(void); //Update Roll Moment

	void resetUpdateFlags_(void);
	
	
};


#endif
