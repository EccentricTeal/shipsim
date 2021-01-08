/*-----------------------------------------------------------------------------
Ship Numerical Model Class


Author: Suisei WADA (D1)
        @Faculty of Naval Architecture and Ocean Engineering, Osaka University
Date:   10th May, 2020
-----------------------------------------------------------------------------*/
#include "mmg_models.hpp"

//Constractor
ShallowWaterModel::ShallowWaterModel()
{
	;
}

//Destructor
ShallowWaterModel::~ShallowWaterModel()
{
	;
}

//Accessor
void ShallowWaterModel::update(void)
{
	updateX_();
	updateY_();
	updateN_();
	updateK_();

	resetUpdateFlags_();
}

double ShallowWaterModel::getForceX(void)
{
	double X = X_.value;
	return X;	
}

double ShallowWaterModel::getForceY(void)
{
	double Y = Y_.value;
	return Y;	
}

double ShallowWaterModel::getForceN(void)
{
	double N = N_.value;
	return N;	
}

double ShallowWaterModel::getForceK(void)
{
	double K = K_.value;
	return K;	
}


//Private Functions
void ShallowWaterModel::calcU_(void)
{
	if(false == tempU_.isUpdated)
	{
		tempU_.value = std::sqrt( std::pow(vel_.linear.x(),2) + std::pow(vel_.linear.y(),2) );
		tempU_.isUpdated = true;
	}
	
}


void ShallowWaterModel::calcF_N_(void)
{
	if(false == tempF_N_.isUpdated)
	{
		if(false == tempU_.isUpdated)
		{
			calcU_();
		}
		double U = tempU_.value;
		double u = vel_.linear.x();
		double v = vel_.linear.y();
		double beta = std::atan( -v / u );
		double D_P = propellerparam_.getDiameter();
		double Pitch = propellerparam_.getPitch();		
		double x_P = propellerparam_.getForcePoint_x();
		double z_P = propellerparam_.getForcePoint_z();
		double x_R = rudderparam_.getForcePoint_x();
		double z_R = rudderparam_.getForcePoint_z();		

		double L = bodyparam_.getLength();
		double D = bodyparam_.getDraft();
		double H_R = rudderparam_.getHeight();
		double W_R = rudderparam_.getWidth();		
		double Eta = D_P / H_R;
		double w_P0 = mmgparam_.get_mmgPropellerParam("w_P0");
		double Gamma_R = mmgparam_.get_mmgRudderParam("Gamma_R");		
		double Epsilon = mmgparam_.get_mmgRudderParam("Epsilon");
		double Kapper = mmgparam_.get_mmgPropellerParam("Kapper");
		double f_alpha = mmgparam_.get_mmgRudderParam("f_Alpha");
		double K_T = propellerparam_.getKT();		

		double A_R = W_R * H_R;
		double x_P_dash = x_P / L;
		double z_P_dash = z_P / L;
		double r_dash = vel_.angular.z() * L / U;
		double phi_dot_dash = vel_.angular.x() * L / U;		
			
		double beta_P = beta - x_P_dash * r_dash + z_P_dash * phi_dot_dash;
		double w_P = w_P0 * (1 - (1 - std::pow(std::cos(beta_P), 2) ) * (1 - std::abs(beta_P)));
		double u_P = ( 1 - w_P ) * vel_.linear.x();
		double propeller_slip = ( 1 - u_P / (propeller_[0].rotate * Pitch) );
		double J_P0 = vel_.linear.x() * (1 - w_P0) / (propeller_[0].rotate * D_P);

		double beta_R = 0.0;
		double u_R = 0.0;
		double v_R = 0.0;
		double squareU_R = 0.0;
		double Alpha_R = 0.0;
		double F_N = 0.0;

		
		beta_R = beta - ( (x_R / L) * (vel_.angular.z() * L / U) ) + ( (z_R / D) * (vel_.angular.x() * D / vel_.linear.y()) );
		v_R = U * Gamma_R * beta_R;
		u_R =
			Epsilon
			* vel_.linear.x()
			* ( 1 - w_P0 )
			* std::sqrt(
				Eta
				* std::pow(1 + (
							   Kapper 
							   * (std::sqrt(1 + (propeller_slip * K_T) / (phyconst_.pi * J_P0 * J_P0) ) - 1 )
							   ),
					2)
				+ ( 1 - Eta )
				);
	
		squareU_R = (u_R * u_R) + (v_R * v_R);
	
		Alpha_R = rudder_[0].angle - std::atan( v_R / u_R );

		F_N = 0.5 * phyconst_.rho * A_R * squareU_R * f_alpha * std::sin(Alpha_R);

		tempF_N_.value = F_N;
		tempF_N_.isUpdated = true;
	}
	
	
}


void ShallowWaterModel::calcX_H_(void)
{
	if(false == tempX_H_.isUpdated)
	{
		double X_H_dash = 0.0;
		double U = 0.0;

		if(false == tempU_.isUpdated)
		{
			calcU_();
		}
		U = tempU_.value;

		double L = bodyparam_.getLength();
		double D = bodyparam_.getDraft();		
		double v = vel_.linear.y() / U;
		double r = vel_.angular.z() * L / U;
		double phi = pos_.angular.x();

		X_H_dash =
			-mmgparam_.get_mmgHullParam("R0")
			+( mmgparam_.get_mmgHullParam("Xvv") * v * v )
			+( mmgparam_.get_mmgHullParam("Xvr") * v * r )
			+( mmgparam_.get_mmgHullParam("Xrr") * r * r )
			+( mmgparam_.get_mmgHullParam("Xvvvv") * std::pow(v, 4) )
			+( mmgparam_.get_mmgHullParam("Xvp") * v * phi )
			+( mmgparam_.get_mmgHullParam("Xrp") * r * phi )
			+( mmgparam_.get_mmgHullParam("Xpp") * phi * phi );
	
		double X_H = 0.5 * phyconst_.rho * L * D * U * U * X_H_dash;
		tempX_H_.value = X_H;
		tempX_H_.isUpdated = true;
	}
	
}


void ShallowWaterModel::calcY_H_(void)
{
	if(false == tempY_H_.isUpdated)
	{
		double Y_H_dash = 0.0;
		double U = 0.0;

		if(false == tempU_.isUpdated)
		{
			calcU_();
		}
		U = tempU_.value;

		double L = bodyparam_.getLength();
		double D = bodyparam_.getDraft();		
		double v = vel_.linear.y() / U;
		double r = vel_.angular.z() * L / U;
		double phi = pos_.angular.x();

		Y_H_dash =
			( mmgparam_.get_mmgHullParam("Yv") * v )
			+( mmgparam_.get_mmgHullParam("Yr") * r )
			+( mmgparam_.get_mmgHullParam("Yvvv") * v * v * v )
			+( mmgparam_.get_mmgHullParam("Yvvr") * v * v * r )
			+( mmgparam_.get_mmgHullParam("Yvrr") * v * r * r )
			+( mmgparam_.get_mmgHullParam("Yrrr") * r * r * r )
			+( mmgparam_.get_mmgHullParam("Yp") * phi )
			+( mmgparam_.get_mmgHullParam("Yvvp") * v * v * phi )
			+( mmgparam_.get_mmgHullParam("Yvpp") * v * phi * phi )
			+( mmgparam_.get_mmgHullParam("Yrrp") * r * r * phi )
			+( mmgparam_.get_mmgHullParam("Yrpp") * r * phi * phi );

		double Y_H = 0.5 * phyconst_.rho * L * D * U * U * Y_H_dash;
		tempY_H_.value = Y_H;
		tempY_H_.isUpdated = true;
	}
				
}


void ShallowWaterModel::calcN_H_(void)
{
	if(false == tempN_H_.isUpdated)
	{		
		double N_H_dash = 0.0;
		double U = 0.0;

		if(false == tempU_.isUpdated)
		{
			calcU_();
		}
		U = tempU_.value;
		double L = bodyparam_.getLength();
		double D = bodyparam_.getDraft();		
		double v = vel_.linear.y() / U;
		double r = vel_.angular.z() * L / U;
		double phi = pos_.angular.x();

		N_H_dash =
			( mmgparam_.get_mmgHullParam("Nv") * v )
			+( mmgparam_.get_mmgHullParam("Nr") * r )
			+( mmgparam_.get_mmgHullParam("Nvvv") * v * v * v )
			+( mmgparam_.get_mmgHullParam("Nvvr") * v * v * r )
			+( mmgparam_.get_mmgHullParam("Nvrr") * v * r * r )
			+( mmgparam_.get_mmgHullParam("Nrrr") * r * r * r )
			+( mmgparam_.get_mmgHullParam("Np") * phi )
			+( mmgparam_.get_mmgHullParam("Nvvp") * v * v * phi )
			+( mmgparam_.get_mmgHullParam("Nvpp") * v * phi * phi )
			+( mmgparam_.get_mmgHullParam("Nrrp") * r * r * phi )
			+( mmgparam_.get_mmgHullParam("Nrpp") * r * phi * phi );

		double N_H = 0.5 * phyconst_.rho * L * D * U * U * N_H_dash;
		tempN_H_.value = N_H;
		tempN_H_.isUpdated = true;
	}
	
}

void ShallowWaterModel::calcX_P_(void)
{
	if(false == tempX_P_.isUpdated)
	{		
		double U = 0.0;

		if(false == tempU_.isUpdated)
		{
			calcU_();
		}
		U = tempU_.value;

		double L = bodyparam_.getLength();
		double D = bodyparam_.getDraft();
		double k0 = mmgparam_.get_mmgPropellerParam("k0");
		double k1 = mmgparam_.get_mmgPropellerParam("k1");
		double k2 = mmgparam_.get_mmgPropellerParam("k2");
		double w_P0 = mmgparam_.get_mmgPropellerParam("wP0");
		double t_0 = mmgparam_.get_mmgPropellerParam("t0");		
		double D_P = propellerparam_.getDiameter();
		double u = vel_.linear.x();
		double v = vel_.linear.y();
		double x_P = propellerparam_.getForcePoint_x();
		double z_P = propellerparam_.getForcePoint_z();		
		
		double x_P_dash = x_P / L;
		double z_P_dash = z_P / D;
		double r = vel_.angular.z() * L / U;
		double phi_dot = vel_.angular.x() * L / U;	
		double beta = std::atan( -v / u );
		double beta_P = beta - x_P_dash * r + z_P_dash * phi_dot;		
		double w_P =  w_P0 * (1 - (1 - std::pow(std::cos(beta_P), 2) ) * (1 - std::abs(beta_P)));
		double J_P = u * (1 - w_P) / (propeller_[0].rotate * D_P);
		
		double T = phyconst_.rho * std::pow(propeller_[0].rotate, 2) * std::pow(D_P, 4) * ( k2 * J_P * J_P + k1 * J_P + k0 );
		double X_P = ( 1 - t_0 ) * T;
		
		tempX_P_.value = X_P;
		tempX_P_.isUpdated = true;
	}
	
}


void ShallowWaterModel::calcX_R_(void)
{
	if(false == tempX_R_.isUpdated)
	{
		double F_N = 0.0;
		if(false == tempF_N_.isUpdated)
		{
			calcF_N_();
		}
		F_N = tempF_N_.value;
		double t_R = mmgparam_.get_mmgPropellerParam("t_R");
				
		double X_R = -(1 - t_R ) * F_N * std::sin( rudder_[0].angle ) * std::cos( pos_.angular.x() );
		tempX_R_.value = X_R;
		tempX_R_.isUpdated = true;
	}
	
}


void ShallowWaterModel::calcY_R_(void)
{
	if(false == tempY_R_.isUpdated)
	{		
		double F_N = 0.0;
		if(false == tempF_N_.isUpdated)
		{
			calcF_N_();
		}
		F_N = tempF_N_.value;
		double alpha_H = mmgparam_.get_mmgPropellerParam("alpha_H");
				
		double Y_R = -(1 + alpha_H ) * F_N * std::cos( rudder_[0].angle ) * std::cos( pos_.angular.x() );	
		tempY_R_.value = Y_R;
		tempY_R_.isUpdated = true;
	}
			
}


void ShallowWaterModel::calcN_R_(void)
{
	if(false == tempN_R_.isUpdated)
	{
		double F_N = 0.0;
		if(false == tempF_N_.isUpdated)
		{
			calcF_N_();
		}
		F_N = tempF_N_.value;
		double alpha_H = mmgparam_.get_mmgPropellerParam("alpha_H");
		double x_H = mmgparam_.get_mmgPropellerParam("x_H");
		double x_R = rudderparam_.getForcePoint_x();		
		
		double N_R =
			- (x_R + alpha_H * x_H )
			* F_N
			* std::cos( rudder_[0].angle )
			* std::cos( pos_.angular.x() );

		tempN_R_.value = N_R;
		tempN_R_.isUpdated = true;
	}
	
}

   
void ShallowWaterModel::updateX_(void)
{
	if(false == X_.isUpdated)
	{
		calcX_H_();
		calcX_P_();
		calcX_R_();
		X_.value = tempX_H_.value + tempX_P_.value + tempX_R_.value;
		X_.isUpdated = true;
	}
}

void ShallowWaterModel::updateY_(void)
{
	if(false == Y_.isUpdated)
	{
		calcY_H_();;
		calcY_R_();
		Y_.value = tempY_H_.value + tempY_R_.value;		
		Y_.isUpdated = true;
	}
}


void ShallowWaterModel::updateN_(void)
{
	if(false == N_.isUpdated)
	{
		calcN_H_();
		calcN_R_();
		N_.value = tempN_H_.value + tempN_R_.value;
		N_.isUpdated = true;
	}
}


void ShallowWaterModel::updateK_(void)
{
	if(false == K_.isUpdated)
	{
		double a = mmgparam_.get_mmgHullParam("a");		
		double b = mmgparam_.get_mmgHullParam("b");
		double K_H = mmgparam_.get_mmgHullParam("K_H");
		double z_R = rudderparam_.getForcePoint_z();		
		double m = massparam_.getMass();
		double Ixx = massparam_.getInertia_xx();
		double Jxx = massparam_.getAddInertia_xx();		
		double K_phi = 0.0;
		double K_phiphi = 0.0;
		double Y_R = 0.0;
		double GM = bodyparam_.getGm();		
		
		K_phi = -2.0 / phyconst_.pi * a * std::sqrt( phyconst_.g * m * GM * ( Ixx + Jxx ) );		
		K_phiphi = -0.75* b * 180 / phyconst_.pi * ( Ixx + Jxx );
		
		if(false == tempY_R_.isUpdated)
		{
			calcY_R_();
		}
		Y_R = tempY_R_.value;		
		
		double K =
			K_H
			- ( Y_R * z_R )
			- ( m * phyconst_.g * GM * pos_.angular.x() )
			+ ( K_phi * vel_.angular.x() )
			+ ( K_phiphi * vel_.angular.x() * std::abs(vel_.angular.x()) );

		K_.value = K;
		K_.isUpdated = true;
	}	
	
}


void ShallowWaterModel::resetUpdateFlags_(void)
{
	X_.isUpdated = false;
	Y_.isUpdated = false;
	N_.isUpdated = false;
	K_.isUpdated = false;

	tempU_.isUpdated = false;
	tempF_N_.isUpdated = false;
	tempX_H_.isUpdated = false;
	tempY_H_.isUpdated = false;
	tempN_H_.isUpdated = false;
	tempX_P_.isUpdated = false;
	tempX_R_.isUpdated = false;
	tempY_R_.isUpdated = false;
	tempN_R_.isUpdated = false;
}

   
