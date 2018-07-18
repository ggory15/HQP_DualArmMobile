# HQP_DualArmMobile
HQP_DualArmMobile for Ubuntu 16.04

This module is to control manipulator or humanoid robot using HQP.
The base of this code is TSID algorithm of Gepetto team in CNRS.

I modified some function and added some tasks for controlling robot safely.
(For instance, task transition algorithm, singularity avoidance algorithm, and self-collision avoidance algorithm)

```Dependency```

Eigen Library

RBDL Library

qpOASES Library

V-REP for simulation

```Cmake Setting```

DoF: the number of Joint

author: ggory15@snu.ac.kr

