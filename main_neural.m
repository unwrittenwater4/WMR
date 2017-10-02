%=============================================================================%
% EE5380 Final Project
% 
% December 2016 
%
% This is matlab simulation of 4 WMR, ackermann driven  with a lidar 
% sensor. It can use both nonlinear back stepping and neural network
% for path error correction. Navigation can be switched between PFN
% wall_follow or bug2.
% Implementation of localization and VHF navigation algorithm not present
% See [link] for detailed report of the implementation
%
% Authors: Singh, Siddharth; Sengupta, Ayush
% Original code snippet author: Dierks, Dr. Travis
%
% Copyright:
%          All copyrights belongs to authors
%          This code is only for illustration and demostration purpose;
%          No part of the code should be copied or reused without prior
%          permission from authors.
%=============================================================================%


close all
clear all
clc
commandwindow;

%End time and simulation sampling time
Tend=500;
Ts=.005;

%PLOT Update Sampling Time
TsP=.25;

%Robot Initial Position
x0=-3;
y0=-3;
th0=-pi/4;

%Robot Goal Position
goal_position=[30;-12];
way_points_x=[x0 5 -30 -5 -5 -33 30 -5 8 30];
way_points_y=[y0 -33 -33 -22 7 27 27 20 -12 -12];

% Start	-3	-3
% A	5	-33
% B	-30	-33
% C	-5	-22
% D	-5	7
% E	-33	27
% F	30	27
% G	-5	20
% H	8	-12
% I	30	-12



%Robot Initial Velocity
V0=[0;0];

%Lidar Update Sampling Time
TsL=.1;

%update Time on Plot in Title
TsPC=2;

%Lidar max sensing distance and sensor coverage
sense_angles= -3*pi/4: (pi/20) : 3*pi/4;
zmax=20;

%Load a map for testing
get_file_name=0; %1 - Ask for to select map; 0 - Load map in filename below
filename='Map 1a.txt';
EE301_load_map;
figure(1)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Set up plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%heading line
rad=0.25;
xline1=[0; 0];
xline2=[1.5*rad; 0];

% Plot robot in default position and store plot handles for updating
circ_numPts= 21;    % Estimate circle as circ_numPts-1 lines
circ_ang=linspace(0,2*pi,circ_numPts);
circ_rad=ones(1,circ_numPts)*rad;
[circ_x circ_y]= pol2cart(circ_ang,circ_rad);
handle_circ= plot(x0+circ_x,y0+circ_y,'b-','LineWidth',1.5);
handle_line= plot(x0+[xline1(1) xline2(1)], ...
                    y0+[xline1(2) xline2(2)],'b-','LineWidth',1.5);
handle_lidar=plot(x0+zmax*cos(sense_angles+th0), ... 
                    y0+zmax*sin(sense_angles+th0),'r-','LineWidth',2);

plot(goal_position(1), goal_position(2),'ro','MarkerSize',10);
plot(x0, y0,'bo','MarkerSize',10);
for i=2:length(way_points_x)-1
    plot(way_points_x(i), way_points_y(i),'ko','MarkerSize',10);
end

plot(way_points_x,way_points_y,'k--');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% laser models:
% model 1: "realistic"
laser_model{1}.p_hit = 0.98;
laser_model{1}.p_short = 0.005;
laser_model{1}.p_max = 0.01;
laser_model{1}.p_rand = 0.005;
laser_model{1}.sigma_hit = 0.0025;
laser_model{1}.lambda_short = 1;
laser_model{1}.z_max = zmax;
laser_model{1}.z_max_eps = 0.02;
laser_model{1}.laser_angles=sense_angles;
laser_temp=laser_model(1);
laser_model{1}.sample=laser_sample_model(laser_temp,1000);

% model 2: noise free
laser_model{2}.p_hit = 1.0;
laser_model{2}.p_short = 0;
laser_model{2}.p_max = 0;
laser_model{2}.p_rand = 0;
laser_model{2}.sigma_hit = 0;
laser_model{2}.lambda_short = 1;
laser_model{2}.z_max = zmax;
laser_model{2}.z_max_eps = 0.02;
laser_model{2}.laser_angles = sense_angles;
laser_temp=laser_model(2);
laser_model{2}.sample=laser_sample_model(laser_temp,10);

% model 3: "Always hit with Gaussian noise"
laser_model{3}.p_hit = .98;
laser_model{3}.p_short = 0;
laser_model{3}.p_max = .02;
laser_model{3}.p_rand = 0;
laser_model{3}.sigma_hit = 0.0025;
laser_model{3}.lambda_short = 1;
laser_model{3}.z_max = zmax;
laser_model{3}.z_max_eps = 0.02;
laser_model{3}.laser_angles=sense_angles;
laser_temp=laser_model(3);
laser_model{3}.sample=laser_sample_model(laser_temp,1000);

%select the laser model by changing the index in laser_model(-)
laser_temp=laser_model(3);

%Actual Robot Position - DO NOT MODIFY
robot_pose=zeros(3,floor(Tend/Ts)+1);
V=ones(2,floor(Tend/Ts)+1);
V(1,:)=V(1,:)*V0(1);
V(2,:)=V(2,:)*V0(2);
Vd=V;
Vddot=Vd;
robot_pose(:,1)=[x0;y0;th0];
desired_robot_pose=robot_pose;

behavior=3;
kg=1;
slow_down=0;
final_goal=0;

kL=0;
kP=0;
kP2=0;
lidar_new=0;
figure(1)
axis([min(min([walls(:,1) walls(:,3)]))-1 max(max([walls(:,1) walls(:,3)])) ...
    +1 min(min([walls(:,2) walls(:,4)]))-1 max(max([walls(:,2) walls(:,4)]))+1])
vidObj = VideoWriter([getenv('USERNAME') ' Exam2.mp4'],'MPEG-4');
open(vidObj);
k=1;
% figure
% hold on
while k<=Tend/Ts && ~final_goal

    
    %Take Lidar Reading
    if k>=TsL/Ts*kL
        
        %generate the sensor measurement based on the real position -
        %results are in the robot reference frame
        z_true=ray_cast(walls, laser_temp, robot_pose(:,k));
        z_meas=laser_meas_model(z_true,laser_temp);
        
        kL=kL+1;
        lidar_new=1;
    end
    
    %Robot Navigation Algorithms
    %Plan route
    goal_position=[way_points_x(kg);way_points_y(kg)];
    if ((robot_pose(1,k)-goal_position(1))^2 ...
         +(robot_pose(2,k)-goal_position(2))^2) < 0.25
        if kg<length(way_points_x)
            kg=kg+1;
        else
            final_goal=1;
        end
    end
    
    if kg==length(way_points_x)
        slow_down=1;
    end
    %Control law to execute plan
    %Robot Navigation Algorithms - This generates the DESIRED velocity
    [Vd(:,k), Vddot(:,k)] = robot_nav( V(:,k),z_meas,sense_angles, ...
                                       behavior,goal_position, ...
                                       robot_pose(:,k),slow_down);
    
    %Get next pose of virtual reference robot
    desired_robot_pose(:,k+1)=diff_drive(desired_robot_pose(:,k),Vd(:,k),Ts);
    
    %Implemented in Take Home exam 1
    Vdot = diff_drive_dyn_neural([robot_pose(:,k)' V(:,k)' ...
                                desired_robot_pose(:,k)' Vd(:,k)' Vddot(:,k)']);
    V(:,k+1)=V(:,k)+Ts*Vdot;
%     plot(Vdot);
    %Get next robot pose - Implemented in homework 1
    robot_pose(:,k+1)=diff_drive(robot_pose(:,k),V(:,k),Ts);
     
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %PLOT RESULTS
    if  k>=TsP/Ts*kP
        
        kP=kP+1;
        
        xline2_plot=[cos(robot_pose(3,k)) -sin(robot_pose(3,k));
            sin(robot_pose(3,k)) cos(robot_pose(3,k))]*xline2;
        
        %Plot Actual Robot Position
        set(handle_circ, 'XData', robot_pose(1,k)+circ_x,'YData', ...
             robot_pose(2,k)+circ_y);
        set(handle_line, 'XData', robot_pose(1,k) ...
            +[xline1(1) xline2_plot(1)],'YData', ...
             robot_pose(2,k)+[xline1(2) xline2_plot(2)]);
        
        %Plot LIDAR Data - Nothing is plot if zmeas==zmax
        if (mean(z_meas)<zmax)
            set(handle_lidar,'Visible','on')
            set(handle_lidar, 'XData', robot_pose(1,k) + ...
                z_meas.*cos(sense_angles+robot_pose(3,k)), 'YData', ...
                 robot_pose(2,k)+z_meas.*sin(sense_angles+robot_pose(3,k)));
        else
            set(handle_lidar,'Visible','off')
        end
        
        if  k>=TsPC/Ts*kP2
            kP2=kP2+1;
            title(['Simulation Time:  ', num2str(k*Ts),' (sec)'],'FontSize',16)
        end
        drawnow
        currFrame = getframe;
        writeVideo(vidObj,currFrame);
        %pause(0.1)
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    k=k+1;
end

plot(robot_pose(1,1:k),robot_pose(2,1:k),'b.-')
currFrame = getframe;
writeVideo(vidObj,currFrame);
close(vidObj);

figure(2)
subplot 211
plot((0:k-1)*Ts, V(1,(1:k)))
xlabel('Time (Sec)', 'FontSize', 16)
ylabel('Linear Velocity (m/s)', 'FontSize', 16)
title('Robot Velocities','FontSize',16)

subplot 212
plot((0:k-1)*Ts,V(2,(1:k)))
xlabel('Time (Sec)', 'FontSize', 16)
ylabel('Angular Velocity (rad/s)', 'FontSize', 16)