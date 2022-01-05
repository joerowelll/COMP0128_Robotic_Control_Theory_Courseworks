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
d = zeros(size(s))
u = zeros(size(s))
e = zeros(size(s))
%Add gaussian noise [See noise function]
s_noisy= noise(s);
Kp = 1
Ki = 1 
Kd = 0
for (t = 0:ts:total_time)
    for (j = 1:size(s))
        e(j) = d_r - d(j);
        e(j) = noise(e(j));
        u(j) =  u(j) + Kp * e(j) + Ki * e(j) + e(j); 
        u(j) = noise(u(j));
        d(j) = d(j) + (s(j)-u(j)) * t;
        disp(d)
        j = j+1;

    end

end

figure
subplot(2,2,1)
plot(d_r,t, d,t)
subplot(2,2,2)


function y = noise(u)
% std = sqrt(var)
mean = 0;
standard_dev = 0.01;
y = u + standard_dev*randn(1)+ mean;
end