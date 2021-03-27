%
% This script loads in data from an experimental step test with both motors set at 
% an analogWrite value of 128. The velocity in in/s is plotted, along with a simulated
% step response for a system with DC gain K and pole manitude sigma set below. This is 
% followed by a plot for the closed loop response for a PI controller
%
data = importdata('ForwardTest128(50mssampletime).txt');
% forward velocity is column 11
rhodot = data(:,11);
% sample time is 50 ms
time = .05*(0:length(rhodot)-1);
figure(1)
plot(time,rhodot);
ylabel('in/sec')
xlabel('sec')
%
% Estimated gain and pole magnitude for first order system model The psudo input
% for this sytem will be pwm1 + pwm2, or 128 + 128
%
K=19.5/(2*128);
sigma = 10;
sys = tf(K*sigma,[1 sigma]);
[y,ts]=step(sys*(2*128));
hold on
plot(ts+1,y);
hold off
legend('Experimental','Simulated')

%
% PI controller
%
Kp=4;
Ki=40;
C = tf([Kp  Ki],[1 0]);
%
% step response of closed loop system
%
figure(2)
step(feedback(sys*C,1))