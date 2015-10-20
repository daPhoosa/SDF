/* 
 * SDF - Second Derivative Filter
 * by Phillip Schmidt
 * v1.0
 *
 */


#include "SDF.h"


// 
SDF::SDF(float _AccelLimit, float _dt)
{
	
	AccelLimit_dt = _AccelLimit * _dt;
	AccelLimitInv_2 = 0.5f / _AccelLimit; // = 1 / (2 * A)		-- to reduce computations
	
	dt = _dt;

	PositionNow = 0;
	VelocityNow = 0;

}

// 
float SDF::in(const float& target) // 78us on 16Mhz 
{
	float VelocityNext;
	float PositionFuture;
	
	// Predict future stopping point if accelerating in current direction
	if(VelocityNow > 0.0f)
	{
		VelocityNext   =  VelocityNow + AccelLimit_dt;
		PositionFuture =  VelocityNext * VelocityNext * AccelLimitInv_2 + PositionNow;	// stopPosition = V^2 / (2 * A) + P
	}
	else
	{
		VelocityNext   =  VelocityNow - AccelLimit_dt;
		PositionFuture = -VelocityNext * VelocityNext * AccelLimitInv_2 + PositionNow;
	}
	
	// set acceleration direction based on predicted future stopping point
	if(PositionFuture < target)
	{
		VelocityNow +=  AccelLimit_dt; // Integrate accel to get velocity
	}
	else
	{
		VelocityNow -= AccelLimit_dt;
	}

	// Integrate vel to get position
	PositionNow += VelocityNow * dt;
	
	return PositionNow;
	
}


float SDF::out()
{
	return PositionNow;
}

