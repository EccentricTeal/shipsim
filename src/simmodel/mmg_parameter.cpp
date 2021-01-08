/*-----------------------------------------------------------------------------
MMG Parameter Structures and Classes


Author: Suisei WADA (D1)
        @Faculty of Naval Architecture and Ocean Engineering, Osaka University
Date:   21th May, 2020
-----------------------------------------------------------------------------*/
#include "mmg_parameter.hpp"
#include <iostream>
#include <exception>
#include <utility>

BodyParam::BodyParam()
{
	setParam(0.0, 0.0, 0.0, 0.0);
}

BodyParam::~BodyParam()
{
	;
}

int BodyParam::setParam(double length, double beam, double draft, double gm)
{
	length_ = length;
	beam_ = beam;
	draft_ = draft;
	gm_ = gm;

	return 1;	
}

double BodyParam::getLength(void)
{
	return length_;
}

double BodyParam::getBeam(void)
{
	return beam_;
}

double BodyParam::getDraft(void)
{
	return draft_;
}

double BodyParam::getGm(void)
{
	return gm_;
}



MassParam::MassParam()
{
	mass_ = 0.0;
	addmass_ = Eigen::Vector3d::Zero();
	inertia_ = Eigen::Matrix3d::Zero();
	addinertia_ = Eigen::Matrix3d::Zero();
	cog_position_ = Eigen::Vector3d::Zero();
}

MassParam::~MassParam()
{
	;
}

int MassParam::setParam(double mass, Eigen::Vector3d addmass, Eigen::Matrix3d inertia, Eigen::Matrix3d addinertia, Eigen::Vector3d cog_position)
{
	mass_ = mass;
	
	addmass_(0,0) = addmass(0,0);	
	addmass_(0,1) = addmass(0,1);
	addmass_(0,2) = addmass(0,2);

	inertia_(0,0) = inertia(0,0);	
	inertia_(0,1) = inertia(0,1);
	inertia_(0,2) = inertia(0,2);
	inertia_(1,0) = inertia(1,0);	
	inertia_(1,1) = inertia(1,1);
	inertia_(1,2) = inertia(1,2);
	inertia_(2,0) = inertia(2,0);	
	inertia_(2,1) = inertia(2,1);
	inertia_(2,2) = inertia(2,2);

	addinertia_(0,0) = addinertia(0,0);	
	addinertia_(0,1) = addinertia(0,1);
	addinertia_(0,2) = addinertia(0,2);
	addinertia_(1,0) = addinertia(1,0);	
	addinertia_(1,1) = addinertia(1,1);
	addinertia_(1,2) = addinertia(1,2);
	addinertia_(2,0) = addinertia(2,0);	
	addinertia_(2,1) = addinertia(2,1);
	addinertia_(2,2) = addinertia(2,2);

	cog_position_(0,0) = cog_position(0,0);	
	cog_position_(0,1) = cog_position(0,1);
	cog_position_(0,2) = cog_position(0,2);	

	return 1;
}

double MassParam::getMass(void)
{
	return mass_;
}

double MassParam::getAddMass_x(void){
	return addmass_(0,0);
}
	
double MassParam::getAddMass_y(void){
		return addmass_(0,1);
}

double MassParam::getAddMass_z(void){
		return addmass_(0,2);
}

double MassParam::getInertia_xx(void){
	return inertia_(0,0);
}

double MassParam::getInertia_xy(void){
	return inertia_(0,1);
}

double MassParam::getInertia_xz(void){
	return inertia_(0,2);
}

double MassParam::getInertia_yx(void){
	return inertia_(1,0);
}

double MassParam::getInertia_yy(void){
	return inertia_(1,1);
}

double MassParam::getInertia_yz(void){
	return inertia_(1,2);
}

double MassParam::getInertia_zx(void){
	return inertia_(2,0);
}

double MassParam::getInertia_zy(void){
		return inertia_(2,1);
}

double MassParam::getInertia_zz(void){
	return inertia_(2,2);
}

double MassParam::getAddInertia_xx(void){
	return addinertia_(0,0);
}

double MassParam::getAddInertia_xy(void){
	return addinertia_(0,1);
}

double MassParam::getAddInertia_xz(void){
	return addinertia_(0,2);
}

double MassParam::getAddInertia_yx(void){
	return addinertia_(1,0);
}

double MassParam::getAddInertia_yy(void){
	return addinertia_(1,1);
}

double MassParam::getAddInertia_yz(void){
	return addinertia_(1,2);
}

double MassParam::getAddInertia_zx(void){
	return addinertia_(2,0);
}

double MassParam::getAddInertia_zy(void){
		return addinertia_(2,1);
}

double MassParam::getAddInertia_zz(void){
	return addinertia_(2,2);
}

double MassParam::getCogPos_x(void){
	return cog_position_(0,0);
}

double MassParam::getCogPos_y(void){
	return cog_position_(0,1);
}

double MassParam::getCogPos_z(void){
	return cog_position_(0,2);
}		



PropellerParam::PropellerParam()
{
	diameter_ = 0.0;	
	pitch_ = 0.0;	
	K_T_ = 0.0;	
	force_point_ = Eigen::Vector3d::Zero();
}

PropellerParam::~PropellerParam()
{
	;
}

int PropellerParam::setParam(double diameter, double pitch, double K_T, Eigen::Vector3d force_point)
{
	diameter_ = diameter;
	pitch_ = pitch;
	K_T_ = K_T;
	force_point_(0,0) = force_point(0,0);
	force_point_(0,1) = force_point(0,1);	
	force_point_(0,2) = force_point(0,2);

	return 1;
}

double PropellerParam::getDiameter(void)
{
	return diameter_;
}

double PropellerParam::getPitch(void)
{
	return pitch_;
}

double PropellerParam::getKT(void)
{
	return K_T_;
}

double PropellerParam::getForcePoint_x(void)
{
	return force_point_(0, 0);
}

double PropellerParam::getForcePoint_y(void)
{
	return force_point_(0, 1);
}

double PropellerParam::getForcePoint_z(void)
{
	return force_point_(0, 2);
}



RudderParam::RudderParam()
{
	area_ = 0.0;
   	f_alpha_ = 0.0;	
	height_ = 0.0;
	width_ = 0.0;
	force_point_ = Eigen::Vector3d::Zero();
}

RudderParam::~RudderParam()
{
	;
}

int RudderParam::setParam(double area, double f_alpha, double height, double width, Eigen::Vector3d force_point)
{
	area_ = area;
	f_alpha_ = f_alpha;
    height_ = height;
	width_ = width;
	force_point_(0,0) = force_point(0,0);
	force_point_(0,1) = force_point(0,1);	
	force_point_(0,2) = force_point(0,2);
	return 1;
}
	
double RudderParam::getArea(void)
{
	return area_;
}
	
double RudderParam::getFAlpha(void)
{
	return f_alpha_;
}
	
double RudderParam::getHeight(void)
{
	return height_;
}

double RudderParam::getWidth(void)
{
	return width_;
}
	
double RudderParam::getForcePoint_x(void)
{
	return force_point_(0,0);
}

double RudderParam::getForcePoint_y(void)
{
	return force_point_(0,1);
}

double RudderParam::getForcePoint_z(void)
{
	return force_point_(0,2);
}		



MmgParam::MmgParam()
{
	hull_.clear();
	propeller_.clear();
	rudder_.clear();	
}

MmgParam::~MmgParam()
{
	;
}

double MmgParam::get_mmgHullParam(std::string key)
{
	double value;
	
	try
	{
		value = hull_.at(key);
		return value;
	}
	catch(std::out_of_range&)
	{
		std::cout
			<< "[ERROR] -"
			<< key
			<< "- is not defined as Hull Force MMG Parameter. Please confirm it!"
			<< std::endl;
		std::terminate();
	}
}

double MmgParam::get_mmgPropellerParam(std::string key)
{
	double value;
	
	try
	{
		value = propeller_.at(key);
		return value;
	}
	catch(std::out_of_range&)
	{
		std::cout
			<< "[ERROR] -"
			<< key
			<< "- is not defined as Propeller Force MMG Parameter. Please confirm it!"
			<< std::endl;
		std::terminate();
	}
}	

double MmgParam::get_mmgRudderParam(std::string key)
{
	double value;
	
	try
	{
		value = rudder_.at(key);
		return value;
	}
	catch(std::out_of_range&)
	{
		std::cout
			<< "[ERROR] -"
			<< key
			<< "- is not defined as Rudder Force MMG Parameter. Please confirm it!"
			<< std::endl;
		std::terminate();
	}
}


double MmgParam::set_mmgHullParam(std::string key, double value)
{	
	//Show Current target parameter
	std::cout
		<< "[HULL] Read parameter: "
		<< key
		<< "......"
		<< std::flush;
		
	auto result = hull_.try_emplace(key, value);
	if(true == result.second)
	{
		std::cout
			<< "Success! Set "
			<< key
			<< " as "
			<< result.second
			<< std::endl;
	}
	else
	{
		std::cout
			<< "ERROR! "
			<< key
			<< " is already defined as "
			<< result.second
			<< "! Confirm Parameter Lists!"
			<< std::endl;
	}

}

double MmgParam::set_mmgPropellerParam(std::string key, double value)
{
	//Show Current target parameter
	std::cout
		<< "[PROPELLER] Read parameter: "
		<< key
		<< "......"
		<< std::flush;
		
	auto result = propeller_.try_emplace(key, value);
	if(true == result.second)
	{
		std::cout
			<< "Success! Set "
			<< key
			<< " as "
			<< result.second
			<< std::endl;
	}
	else
	{
		std::cout
			<< "ERROR! "
			<< key
			<< " is already defined as "
			<< result.second
			<< "! Confirm Parameter Lists!"
			<< std::endl;
	}

}

double MmgParam::set_mmgRudderParam(std::string key, double value)
{
	//Show Current target parameter
	std::cout
		<< "[RUDDER] Read parameter: "
		<< key
		<< "......"
		<< std::flush;
		
	auto result = rudder_.try_emplace(key, value);
	if(true == result.second)
	{
		std::cout
			<< "Success! Set "
			<< key
			<< " as "
			<< result.second
			<< std::endl;
	}
	else
	{
		std::cout
			<< "ERROR! "
			<< key
			<< " is already defined as "
			<< result.second
			<< "! Confirm Parameter Lists!"
			<< std::endl;
	}

}	
	
