// IMU Class Header File

#ifndef IMU_H
#define IMU_H

#include <cmath>

class IMU
{
public:
    double Q[4]; // quarternion array
    double dT;   // timestep

    IMU(double q[4], double tstep); // constructor

    void propagate(double (&q)[4], double (&gyro)[3]);
    void normalize(double (&q)[4]);
    void update(double (&q)[4], double (&gyro)[3]);
};

#endif