#ifndef FCL_MODEL_H
#define FCL_MODEL_H

#include "fcl_utility.h"
using namespace fcl;
using namespace std;




class FCL_MODEL
{
public:

    void Initialize_Model();
    void Calculate_Distance();
    void Check_Collision_Pairs();


public:
    fcl::BVHModel<OBB> m[7];
    std::vector<Vec3f> p[7]; 
    std::vector<Triangle> t[7];
    fcl::Transform3f tf[7];
    fcl::Matrix3f Rot[7];
    fcl::Matrix3f Init_Rot[7];
    Vec3f Trs[7];
    Vec3f Init_Trs[7];
    FCL_REAL temp[3];
    fcl::DistanceRes Dis[10];
};





#endif