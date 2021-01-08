/*-----------------------------------------------------------------------------
MMG Model Parameter Structure Definition


Author: Suisei WADA (D1)
        @Faculty of Naval Architecture and Ocean Engineering, Osaka University
Date:   10th May, 2020
-----------------------------------------------------------------------------*/
#ifndef MMG_PARAMETER_HPP
#define MMG_PARAMETER_HPP

//Include 
#include <Eigen/Core>
#include <unordered_map>
#include <string>


/*------------------------------------------------
MMG Ship Unique Parameter
------------------------------------------------*/
class BodyParam
{
	public:
	BodyParam();
	~BodyParam();	
	int setParam(double length, double beam, double draft, double gm);
	double getLength(void);
	double getBeam(void);
	double getDraft(void);
	double getGm(void);

	private:
	double length_; //L [m]
	double beam_; //B [m]
	double draft_; //d [m]
	double gm_; //GM [m]
	
};


class MassParam
{
	public:
	MassParam();
	~MassParam();
	int setParam(double mass, Eigen::Vector3d addmass, Eigen::Matrix3d inertia, Eigen::Matrix3d addinertia, Eigen::Vector3d cog_position);	
	double getMass(void);
	double getAddMass_x(void);
	double getAddMass_y(void);
	double getAddMass_z(void);
	double getInertia_xx(void);
	double getInertia_xy(void);
	double getInertia_xz(void);
	double getInertia_yx(void);
	double getInertia_yy(void);
	double getInertia_yz(void);
	double getInertia_zx(void);
	double getInertia_zy(void);
	double getInertia_zz(void);
	double getAddInertia_xx(void);
	double getAddInertia_xy(void);
	double getAddInertia_xz(void);
	double getAddInertia_yx(void);
	double getAddInertia_yy(void);
	double getAddInertia_yz(void);
	double getAddInertia_zx(void);
	double getAddInertia_zy(void);
	double getAddInertia_zz(void);
	double getCogPos_x(void);
	double getCogPos_y(void);
	double getCogPos_z(void);
	
	private:
	double mass_; //m [kg]
	Eigen::Vector3d addmass_; //(m_x, m_y, m_z) [kg]
	Eigen::Matrix3d inertia_; //(Ixx, Ixy, Ixz / Iyx, Iyy, Iyz, / Izx, Izy, Izz) [kg*m^2]
	Eigen::Matrix3d addinertia_; //(Jxx, Jxy, Jxz / Jyx, Jyy, Jyz, / Jzx, Jzy, Jzz) [kg*m^2]
	Eigen::Vector3d cog_position_; //x_G, y_G, z_G (Midship origin, Coordinate: On Ship)
	
};



class PropellerParam
{	//T = ρ * n_P^2 * D_P^4 * K_T
	
	public:
	PropellerParam();
	~PropellerParam();
	int setParam(double diameter, double pitch, double K_T, Eigen::Vector3d force_point);
	double getDiameter(void);
	double getPitch(void);
	double getKT(void);
	double getForcePoint_x(void);
	double getForcePoint_y(void);
	double getForcePoint_z(void);	
	
	private:
	double diameter_; //D_P
	double pitch_; //P
	double K_T_; //Coef. of Propeller Specification
	Eigen::Vector3d force_point_;	//Point of propeller force(Midship origin, Coordinate: On ship)
		
};

	

class RudderParam
{//F_N = 1/2 * ρ * A_R * U_R^2 * f_α * sin(α_R)

	public:
	RudderParam();
	~RudderParam();
	int setParam(double area, double f_alpha, double height, double width, Eigen::Vector3d force_point);
	double getArea(void);
	double getFAlpha(void);
	double getHeight(void);
	double getWidth(void);
	double getForcePoint_x(void);
	double getForcePoint_y(void);
	double getForcePoint_z(void);
	
	private:
	double area_; //A_R : Area of the rudder
	double f_alpha_; //Rudder Pressure Gradient Coef.
	double height_; //Hieght of Rudder Boad
	double width_; //Width of Rudder Boad
	Eigen::Vector3d force_point_; //Point of rudder force(Midship origin, Coordinate: On ship)
	
};



/*------------------------------------------------
MMG Hydrodynamic Force(Hull) Parameter
------------------------------------------------*/
class MmgParam
{
	public:
	//Constractor, Destructor
	MmgParam();
	~MmgParam();

	//Accessor
	double get_mmgHullParam(std::string key);
	double get_mmgPropellerParam(std::string key);
	double get_mmgRudderParam(std::string key);
	
	double set_mmgHullParam(std::string key, double value);
	double set_mmgPropellerParam(std::string key, double value);
	double set_mmgRudderParam(std::string key, double value);

	
	private:
	std::unordered_map<std::string, double> hull_;
	std::unordered_map<std::string, double> propeller_;
	std::unordered_map<std::string, double> rudder_;
	
};


struct FlagParam
{
	bool isUpdated;
	double value;

	FlagParam():
		isUpdated(false),
		value(0.0)
	{
		;
	}
	
	
};

	

#endif
