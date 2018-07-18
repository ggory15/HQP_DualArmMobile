#ifndef HAPTIC_H_
#define HAPTIC_H_

#include <Eigen/Dense>

using namespace Eigen;

struct hapticData
{
    struct readData // read from haptic
    {
    Vector3d p; // position
    Vector3d force; // force
    Vector3d torque; // joint torque
    Vector3d q; // joint angle
    bool button;
    }read;

    struct writeData    // write to haptic
    {
    // WRITE in controller
    Vector3d force;
	double viscosity;
    }write;
};

#endif
