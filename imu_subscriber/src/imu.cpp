// IMU Class Implementation

#include <iostream>
#include "imu_subscriber/imu.h"

using namespace std;

IMU::IMU(double q[4], double tstep) // constructor declaration
{
    Q[0] = q[0]; // provide initial values to quaternion class variable when instance declared
    Q[1] = q[1];
    Q[2] = q[2];
    Q[3] = q[3];

    dT = tstep; // provide value to timestep class variable when instance declared
}

void IMU::propagate(double (&q)[4], double (&gyro)[3])
{
    q[0] = (Q[0] + (Q[1] * -0.5 * gyro[0] * dT) + (Q[2] * -0.5 * gyro[1] * dT) + (Q[3] * -0.5 * gyro[2] * dT)); // filter calculations using class variable
    q[1] = ((Q[0] * 0.5 * gyro[0] * dT) + Q[1] + (Q[2] * 0.5 * gyro[2] * dT) + (Q[3] * -0.5 * gyro[1] * dT));
    q[2] = ((Q[0] * 0.5 * gyro[1] * dT) + (Q[1] * -0.5 * gyro[2] * dT) + Q[2] + (Q[3] * 0.5 * gyro[0] * dT));
    q[3] = ((Q[0] * 0.5 * gyro[2] * dT) + (Q[1] * 0.5 * gyro[1] * dT) + (Q[2] * -0.5 * gyro[0] * dT) + Q[3]);
}

void IMU::normalize(double (&q)[4]) // normalize function
{
    double temp;
    double mag;

    for (int i = 0; i < 4; i++)
    {
        temp += pow(*(q + i), 2); // add squares of 4 column elements together
    }

    mag = sqrt(temp); // find magnitude

    for (int i = 0; i < 4; i++)
    {
        *(q + i) = *(q + i) / mag; // divide the current contents by the mag and resave
    }
}

void IMU::update(double (&q)[4], double (&gyro)[3]) // update filter function
{
    propagate(q, gyro); // return pointer to calculated q array

    normalize(q); // normalize array

    Q[0] = q[0]; // assign new array (q[4] in main scope) to class variable indices
    Q[1] = q[1];
    Q[2] = q[2];
    Q[3] = q[3];
}