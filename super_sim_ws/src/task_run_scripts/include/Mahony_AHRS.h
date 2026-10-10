#ifndef __MAHONY_AHRS_H
#define __MAHONY_AHRS_H


class MahonyAHRS
{
    //----------------------------------------------------------------------------------------------------
    // Variable declaration
    public:
        MahonyAHRS();
        ~MahonyAHRS();
        void init();
        volatile double  q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;						// quaternion of sensor frame relative to auxiliary frame
        volatile double integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
        //---------------------------------------------------------------------------------------------------
        // Function declarations

        void MahonyAHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz , double cycle);
        void MahonyAHRSupdateIMU(double gx, double gy, double gz, double ax, double ay, double az , double cycle);
};


#endif

