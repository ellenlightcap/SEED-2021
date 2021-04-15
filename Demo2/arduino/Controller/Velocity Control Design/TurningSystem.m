%
% This script loads in data from an experimental step test with both motors set at 
% an analogWrite value of 128, but the left motor (motor 2) negative. The velocity in in/s is plotted, along with a simulated
% step response for a system with DC gain K and pole manitude sigma set below. This is 
% followed by a plot for the closed loop response for a PI controller
%
data = importdata('TurnLeftNegative(128)(50msSample).txt');
% rotational velocity is column 11
phidot = data(:,12);
% sample time is 50 ms
time = .05*(0:length(phidot)-1);
figure(1)
plot(time,phidot);
ylabel('rad/sec')
xlabel('sec')
%
% Estimated gain and pole magnitude for first order system model. The psudo input
% for this sytem will be pwm1 - pwm2, or 128 - (-120)
%
K=2.4/(2*128);
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
Kp=20;
Ki=200;
C = tf([Kp  Ki],[1 0]);
%
% step response of closed loop system
%
figure(2)
step(feedback(sys*C,1))
