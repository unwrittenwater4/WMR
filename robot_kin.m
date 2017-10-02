function [robot_pose_out]=robot_kin(robot_pose,V,Ts)
%Kinematics:  Ackermann WMR Rear Wheel Drive
% Not completely implemented

phi=robot_pose(4);
l=1;
v=V(1);
w=V(2);

k00=cos(phi)*v;
k01=sin(phi)*v;
k02=(tan(phi)/l)*v;
k03=w;

k10=cos(phi+Ts*k03/2)*v;
k11=sin(phi+Ts*k03/2)*v;
k12=(tan(phi)/l+Ts*k03/2)*v;
k13=w;

k20=cos(phi+Ts*k13/2)*v;
k21=sin(phi+Ts*k13/2)*v;
k22=(tan(phi)/l+Ts*k13/2)*v;
k23=w;

k30=cos(phi+Ts*k22)*v;
k31=sin(phi+Ts*k22)*v;
k32=(tan(phi)/l+Ts*k23)*v;
k33=w;

robot_pose_out(1)=robot_pose(1)+Ts*(k00+2*(k10+k20)+k30)/6;
robot_pose_out(2)=robot_pose(2)+Ts*(k01+2*(k11+k21)+k31)/6;
robot_pose_out(3)=robot_pose(3)+Ts*(k02+2*(k12+k22)+k32)/6;
robot_pose_out(4)=robot_pose(4)+Ts*(k03+2*(k13+k23)+k33)/6;

robot_pose_out(4)=atan2(sin(robot_pose_out(4)),cos(robot_pose_out(4)));