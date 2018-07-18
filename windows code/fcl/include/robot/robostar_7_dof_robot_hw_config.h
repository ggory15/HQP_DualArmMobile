#ifndef  _ROBOSTAR_6_DOF_ROBOT_HW_CONFIG_H
#define _ROBOSTAR_6_DOF_ROBOT_HW_CONFIG_H

#define DOF 7


#define CNT_TO_RAD_14 (3.141592*2/20971520)
#define CNT_TO_RAD_57 (3.141592*2/13107200)

#define RAD_TO_CNT_14 (1/(CNT_TO_RAD_14))
#define RAD_TO_CNT_57 (1/(CNT_TO_RAD_57))


const double CNT2RAD[DOF] =
{
    CNT_TO_RAD_14,
    CNT_TO_RAD_14,
    CNT_TO_RAD_14,
    CNT_TO_RAD_14,
    CNT_TO_RAD_57,
    CNT_TO_RAD_57,
    CNT_TO_RAD_57,
};
const double RAD2CNT[DOF] =
{
    RAD_TO_CNT_14,
    RAD_TO_CNT_14,
    RAD_TO_CNT_14,
    RAD_TO_CNT_14,
    RAD_TO_CNT_57,
    RAD_TO_CNT_57,
    RAD_TO_CNT_57,
};
const double RATEDTORQUE[DOF] =
{
	0.856 * 160,
    0.856 * 160,
    0.429 * 160,
    0.429 * 160,
	0.193 * 100,
	0.193 * 100,
	0.193 * 100
};

const double OFFSET[DOF] =
{
    105 * DEG2RAD,
    4 * DEG2RAD,
    1 * DEG2RAD,
    -2 * DEG2RAD,
    3 * DEG2RAD,
    3 * DEG2RAD,
    0 * DEG2RAD,
};

#endif // ! _ROBOSTAR_6_DOF_ROBOT_HW_CONFIG_H
