#include "../include/Mahony_AHRS.h"
#include  <math.h>
#include  <iostream>

MahonyAHRS::MahonyAHRS()
{

}

MahonyAHRS::~MahonyAHRS()
{

}

void MahonyAHRS::init()
{
	q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;						// quaternion of sensor frame relative to auxiliary frame
	integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	
}

double limit (double data ,double max , double min)
{
	if(data<=max && data>=min){
		return data;
	}
	else{
		if(data>max)
			return max;
		else if (data < min)
			return min;
	}
}

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
void MahonyAHRS::MahonyAHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz , double cycle) {
	double recipNorm;
  	double q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	double hx, hy, bx, bz;
	double halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	double halfex, halfey, halfez;
	double qa, qb, qc;
	double twoKp = 10.0;											// 2 * proportional gain (Kp)
	double twoKi = 1.0;											// 2 * integral gain (Ki)

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRS::MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az ,cycle);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = 1/sqrt((ax * ax + ay * ay + az * az));
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = 1/sqrt((mx * mx + my * my + mz * mz));
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrt(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * cycle;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * cycle;
			integralFBz += twoKi * halfez * cycle;
			integralFBx = limit(integralFBx,0.01,-0.01) ;
			integralFBy = limit(integralFBy,0.01,-0.01) ;
			integralFBz = limit(integralFBz,0.01,-0.01) ;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * cycle);		// pre-multiply common factors
	gy *= (0.5f * cycle);
	gz *= (0.5f * cycle);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = 1/sqrt((q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3));
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRS::MahonyAHRSupdateIMU(double gx, double gy, double gz, double ax, double ay, double az ,double cycle) {
	double recipNorm =0;
	double halfvx, halfvy, halfvz =0;
	double halfex, halfey, halfez =0;
	double qa, qb, qc =0;
	double  twoKp = 10.0;											// 2 * proportional gain (Kp)
	double  twoKi = 1.0;											// 2 * integral gain (Ki)
	// std::cout<< "time: " <<  cycle << std::endl;
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

		// Normalise accelerometer measurement
		recipNorm = 1/sqrt((ax * ax + ay * ay + az * az));
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx =  q1 *  q3 -  q0 *  q2;
		halfvy =  q0 *  q1 +  q2 *  q3;
		halfvz =  q0 *  q0 - 0.5 +  q3 *  q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * cycle;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * cycle;
			integralFBz += twoKi * halfez * cycle;
			integralFBx = limit(integralFBx,0.01,-0.01) ;
			integralFBy = limit(integralFBy,0.01,-0.01) ;
			integralFBz = limit(integralFBz,0.01,-0.01) ;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0;	// prevent integral windup
			integralFBy = 0.0;
			integralFBz = 0.0;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5 * cycle);		// pre-multiply common factors
	gy *= (0.5 * cycle);
	gz *= (0.5 * cycle);
	qa =  q0;
	qb =  q1;
	qc =  q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = 1/sqrt(( q0 *  q0 +  q1 *  q1 +  q2 *  q2 +  q3 *  q3));
	 q0 *= recipNorm;
	 q1 *= recipNorm;
	 q2 *= recipNorm;
	 q3 *= recipNorm;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
