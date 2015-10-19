/* 
 * SDF.h
 *
 */


#ifndef SDF_h
#define SDF_h

#include <arduino.h>

class SDF
{
	public:
	
		SDF(float _AccelLimit, float _dt);
		float in(const float& target);
		float out();
		

	private:
	
		float AccelLimit_dt;
		float AccelLimitInv_2;
		
		float PositionNow, PositionFuture;
		float VelocityNow;
		float dt;

};

#endif
