# HQP_DualArmMobile

This is the continous transition algorithm using Hierarchical Qudratic Programming(HQP).
The abstract is here: http://dyros.snu.ac.kr/HQPtasks

This code is based on https://github.com/stack-of-tasks/tsid. 
Based on this code, I rewrote the code in order to use in Windows and Linux with V-Rep.

This code consists of four types.

1) V-Rep with 7-DoF arm (branch: fixed-robot)

2) V-Rep with mobile based with single arm (branch: MobileManipulator)

3) V-Rep with dual arm mobile manipulator (branch: DualArmManipulator) in Windows.

4) V-Rep with dual arm mobile manipulator (branch: Ubuntu16.04) in Linux.

You can find the controller for real-time in https://github.com/ggory15/HQP_Robostar. 

This code is operating well by itself, but I didn't upload the final source code until accepting my paper. So if you want to use the code with final version during under review, plz send the mail to me (ggory15@snu.ac.kr)

Dependancy: qpOASES (for solgving QP), Eigen (for calculating linear algebra), RBDL (for getting a robot dynamics).
