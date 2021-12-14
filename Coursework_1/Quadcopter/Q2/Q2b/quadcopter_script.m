%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%  Code modified by Jason Zhang, Ela Kanani and Joseph Rowell for
%%%%  Coursework Q2
%%%%  Visualisation code for quadcopter 
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
clc;
close all;

%Define total width, length and height of flight arena (metres)
spaceDim = 20;
spaceLimits = [-spaceDim/2 spaceDim/2 -spaceDim/2 spaceDim/2 0 spaceDim/2];

%do you want to draw a ground image on the figure?
draw_ground = false;
if(draw_ground)
    ground_img = imread('ground.png');
end


%figure to display drone simulation
f1 = figure;
ax1 = gca;
view(ax1, 3);
axis equal;
axis(spaceLimits)
grid ON
grid MINOR
caxis(ax1, [0 spaceDim]);
hold(ax1,'on')
axis vis3d

%%
num_drones = 1;

time_interval = 0.05;

%instantiate a drone object, input the axis and arena limits
drones = [];
for i = 1:num_drones
    drones = [drones Drone(ax1, spaceDim, num_drones, time_interval)];
end

drones(1).equ_inputs = solveInputs(zeros(1,3),drones(i));

while(drones(1).time < 8.0)
    %clear axis
    cla(ax1);
    
    %update and draw drones
    for i = 1:num_drones
        update(drones(i));
    end
    
    %optionally draw the ground image
    if(draw_ground)
        imagesc([-spaceDim,spaceDim],[-spaceDim,spaceDim],ground_img);
    end
    
    %apply fancy lighting (optional)
    camlight
    
    %update figure
    drawnow
    pause(0.01)
end

%plots to compare change in position and orientation over time.

%import positions and orientations over time from q1
xyz_posQ1 = readmatrix('q1_xyzpos.txt');
orientationQ1 = readmatrix('q1_orientations.txt');
timeQ1 = readmatrix('q1_simtimes.txt'); %CHECK IF THIS IS THE SAME FOR BOTH QS BEFORE RUNNING

figure;
% plot position variation over time

subplot(3,2,1);
plot(timeQ1,xyz_posQ1(1,:),'k'); %Q1
hold on;
plot(timeQ1,drones.xyzpos(1,:),'r--');
title('Variation of x (m) Coordinate Over Time (s)');
legend('Non-Linear','LTI','Location','best')
xlabel('Time (s)')
ylabel('x: Distance (m) from origin')
grid on;

subplot(3,2,3);
plot(timeQ1,xyz_posQ1(2,:),'k'); %Q1
hold on;
plot(timeQ1,drones.orientation(2,:),'r--');
title('Variation of y (m) Coordinate Over Time (s)');
legend('Non-Linear','LTI','Location','best')
xlabel('Time (s)')
ylabel('y: Distance (m) from origin')
grid on;

subplot(3,2,5);
plot(timeQ1,xyz_posQ1(3,:),'k'); %Q1
hold on;
plot(timeQ1,drones.xyzpos(3,:),'r--');
title('Variation of z (m) Coordinate Over Time (s)');
legend('Non-Linear','LTI','Location','best')
xlabel('Time (s)')
ylabel('z: Distance (m) from origin')
grid on;

% plot orientation variation over time
subplot(3,2,2);
plot(timeQ1,orientationQ1(1,:),'k'); %Q1
hold on;
plot(timeQ1,drones.orientation(1,:),'r--');
legend('Non-Linear','LTI','Location','best')
title('Variation of Roll Angle (°) Over Time (s)');
xlabel('Time (s)')
ylabel('Roll Angle (°)')
grid on;

subplot(3,2,4);
plot(timeQ1,orientationQ1(2,:),'k'); %Q1
hold on;
plot(timeQ1,drones.orientation(2,:),'r--');
legend('Non-Linear','LTI','Location','best')
title('Variation of Pitch Angle (°) Over Time (s)');
xlabel('Time (s)')
ylabel('Pitch Angle (°)')
grid on;

subplot(3,2,6);
plot(timeQ1,orientationQ1(3,:),'k'); %Q1
hold on;
plot(timeQ1,drones.orientation(3,:),'r--');
legend('Non-Linear','LTI','Location','best')
title('Variation Yaw Angle (°) Over Time (s)');
xlabel('Time (s)')
ylabel('Yaw Angle (°)')
grid on;

figure
plot3(drones.xyzpos(1,:),drones.xyzpos(2,:),drones.xyzpos(3,:))
hold on
plot3(xyz_posQ1(1,:),xyz_posQ1(2,:),xyz_posQ1(3,:))
hold on
legend('Non-Linear','LTI','Location','best')
title('3D Plot of Quadcopter Trajectory Q2')
xlabel('x')
ylabel('y')
zlabel('z')
grid on
