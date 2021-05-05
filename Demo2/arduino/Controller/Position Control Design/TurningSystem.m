%
% This script loads in data from an experimental step test with angular velocity setupoint
% of 1 rad/s. The actual velocity in in/s is plotted, along with a simulated
% step response for a system with DC gain K and pole manitude sigma set below. This is 
% followed by a plot for the closed loop response for a PI controller
%
data = importdata('velocityStep.txt');
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
K=1;
zeta=.8;
omegan = 2;
sys = tf(K*omegan^2,[1 2*zeta*omegan omegan^2]);
[y,ts]=step(sys);
hold on
plot(ts,y);
hold off
legend('Experimental','Simulated')


%
% PD controller
%
Kp=0.6*1.95;
Kd=0.6;
C = tf([Kd  Kp],[1]);
%
% step response of closed loop system
%
figure(2)
step(feedback(sys*tf([1],[1 0])*C,1))
