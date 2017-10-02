function [robot_pose_out]=diff_drive(robot_pose,V,Ts)

th=robot_pose(3);

v=V(1);
w=V(2);


% RK4
k00=cos(th)*v;
k01=sin(th)*v;
k02=w;

k10=cos(th+Ts*k02/2)*v;
k11=sin(th+Ts*k02/2)*v;
k12=w;

k20=cos(th+Ts*k12/2)*v;
k21=sin(th+Ts*k12/2)*v;
k22=w;

k30=cos(th+Ts*k22)*v;
k31=sin(th+Ts*k22)*v;
k32=w;

robot_pose_out(1)=robot_pose(1)+Ts*(k00+2*(k10+k20)+k30)/6;
robot_pose_out(2)=robot_pose(2)+Ts*(k01+2*(k11+k21)+k31)/6;
robot_pose_out(3)=robot_pose(3)+Ts*(k02+2*(k12+k22)+k32)/6;

robot_pose_out(3)=atan2(sin(robot_pose_out(3)),cos(robot_pose_out(3)));