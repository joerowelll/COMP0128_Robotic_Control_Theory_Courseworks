%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos 
%%%%  Code modified by Jason Zhang, Ela Kanani and Joseph Rowell for
%%%%  coursework Q1
%%%%  Visualisation code for quadcopter 
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
clc;
close all;

%Define total width, length and height of flight arena (metres)
spaceDim = 50;
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

num_drones = 1;

time_interval = 0.05;

%Instantiate a drone object, input the axis and arena limits
drones = [];
for i = 1:num_drones
    drones = [drones Drone(ax1, spaceDim, num_drones, time_interval)];
end
%Inputs to the drone are mg/4 to balance gravity, solveInputs for zero acc
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

figure(2);
% plot position variation over time
subplot(3,2,1);
plot(drones.times,drones.xyzpos(1,:));
title('Variation of x (m) Coordinate Over Time (s)');
xlabel('Time (s)')
ylabel('x: Distance (m) from origin')
grid on;

subplot(3,2,3);
plot(drones.times,drones.xyzpos(2,:));
title('Variation of y (m) Coordinate Over Time (s)');
xlabel('Time (s)')
ylabel('y: Distance (m) from origin')
grid on;

subplot(3,2,5);
plot(drones.times,drones.xyzpos(3,:));
title('Variation of z (m) Coordinate Over Time (s)');
xlabel('Time (s)')
ylabel('z: Distance (m) from origin')
grid on;

% plot orientation variation over time
subplot(3,2,2);
plot(drones.times,drones.orientation(1,:));
title('Variation of Roll Angle (°) Over Time (s)');
xlabel('Time (s)')
ylabel('Roll Angle (°)')
grid on;

subplot(3,2,4);
plot(drones.times,drones.orientation(2,:));
title('Variation of Pitch Angle (°) Over Time (s)');
xlabel('Time (s)')
ylabel('Pitch Angle (°)')
grid on;

subplot(3,2,6);
plot(drones.times,drones.orientation(3,:));
title('Variation Yaw Angle (°) Over Time (s)');
xlabel('Time (s)')
ylabel('Yaw Angle (°)')
grid on;

figure
plot3(drones.xyzpos(1,:),drones.xyzpos(2,:),drones.xyzpos(3,:))
title('3D Plot of Quadcopter Trajectory Q1')
xlabel('x')
ylabel('y')
zlabel('z')
grid on;

%Write xyz positions and roll, pitch and yaw over time to csv file to
%extract for question 2
writematrix(drones.xyzpos, 'q1_xyzpos.txt');
writematrix(drones.orientation,'q1_orientations.txt');
writematrix(drones.times, 'q1_simtimes.txt');