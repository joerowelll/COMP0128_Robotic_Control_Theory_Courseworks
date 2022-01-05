%% TODO
%Make signal
%Add noise
%Run signal through discrete transfer fcn and feedback with noise
%Calculate error
%Input error to pid from d_R
%PID output to transfer function
%plot 



%Make Signal
ts = 0.2
total_time = 16
%s = [0, 0.1, 0.2, 0.3, 0.4, 0.4, 0.4, 0.4, 0.4, 0.3, 0.2,0.2, 0.2, 0.2, 0.2, 0.2, 0.2];
s = [];
s = [s,0:0.02:0.4];
s = [s, linspace(0.4,0.4,4/ts)];
s = [s, linspace(0.4,0.2,2/ts)];
s = [s, linspace(0.2,0.2,6/ts)];

%Make Time, 16s duration with 0.2s intervals
t = 0:ts:total_time;

%Reference distance
d_r = 1;

%Add gaussian noise [See noise function]
s_noisy= noise(s);


%Generate Discrete transfer fcn PID
sys_discrete = tf([11 -9], [10 -10], ts);

%Generate Continous transfer fcn integrator
sys_cont = tf([1],[1 0])

%Run signal through continous transfer function integrator
[d,t] = lsim(sys_cont, s_noisy,t);

%Find error in signal
err = d_r - d ;
noisy_err = noise(err)

%Run signal through discrete trasnfer fcn PID
[s_1,t] = lsim(sys_discrete, d_r + noisy_err ,t)
%Run signal 
[d,t] = lsim(sys_cont, s_1,t)

%WHAT CONVERGENCE LIMIT DO YOU SET?


%Plot
figure
subplot(2,2,1)
[x,y] = lsim(sys_cont, s_1, t)
plot(y,x)
title('Output Distance')
xlabel('Time (seconds)')
ylabel('Distance')
hold on
plot(t, ones(1, length(t)))
%legend('Output distance', 'd_r')

subplot(2,2,2)
lsim(sys_discrete, d_r + noisy_err , t)
title('Output Velocity')
xlabel('Time (seconds)')
ylabel('Velocity')

subplot(2,2,3)
plot(t,s)
title('Input Velocity')
xlabel('Time (seconds)')
ylabel('Velocity')

subplot(2,2,4)
lsim(sys_cont, s_1, t)
title('Output Distance lsim')
xlabel('Time (seconds)')
ylabel('Distance')


function u = pid(err,dt, e_prev)
ei = err * dt;
ed = (err - e_prev )/ dt;
u  = Kp*err + Ki*ei + Kd*ed;
end

function y = noise(u)
% std = sqrt(var)
mean = 0;
standard_dev = 0.01;
y = u + standard_dev*randn(1)+ mean;
end