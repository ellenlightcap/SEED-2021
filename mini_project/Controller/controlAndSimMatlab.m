%% Mini Project: Control Design and Method
% My method to create the control system for the Mini Project occurred
% through a variety of steps. 
% 
% # The first action I took to create the control system was to determine
% my gain or k value. I did this by creating a step response experiment in
% arduino code where I calculated the angular velocity of the wheel when it 
% is at its highest speed. I determined that my anglular velocity was 
% between 0.16 and 0.2 radians per second. There was a little change in my angular velocity because of my usage of the encoder.h library.
% To determine my K or my gain value, I then divided the angular velocity 
% by 5 volt because 5 volts is equivalent to 255 pwm.
% # From my gain value, I created a simulink block diagram to esitmate my
% transfer function. Since I already had my gain, I only had to determine
% my omega value. I determined omega to be 60 inverse volts. Therefore, I
% had created my transfer function that I would utilize in my closed loop
% system.
% # Once I had my transfer function, I created my closed loop controller
% model. I had to determine my kp and ki values as I only used a PI
% controller. I determined my kp and ki values by using the PI tuner. I
% wanted to ensure that my control model had an overshoot under 12% and a
% rise time of a second or less. Therefore, I determined my kp value to
% around 63 pwm per radian. My ki value was about 24 pwm per radian per
% second. Once I had my controller finalized, I actively applied my values
% to my arduino code to see how it worked.
% # After tweaking with my applied arduino controller, I found that a kp
% value of 5 pwm per radian was most efficient. Furthermore, a ki value of
% 0.18 pwm per radian per second helped with integrator wind up and
% lessened overshoot. My values were widely different because of scaling
% and unit conversion factors. Furthermore, I could have made my kp value a
% little larger, probably around 6 to decrease the rise time. Next time, I
% will have a larger kp value. 
%
% *My simulated open and closed loop systems are below. My experimental open
% and closed loop systems are after this document.*
%% Open Loop Step Response
% 
k=0.033; %(rad*volt)/sec
omega = 60;%1/volt
out = sim('transferfunctionmodel');
plot(out.miniTF);
title('Simulated Open Loop Response');
ylabel('Angular Velocity(rad/sec)');
% *Disscussion: Simulated vs Experimental*
% My simulated and experimental values for my open loop step response
% experiment are ratehr similar. My experimental values have a slower rise
% time and have some variance in their output. The slower rise time is due
% to sampling through arduino code. The variance in angular velocity ouput
% changes because of the encoder.h library. Next time, I will use a system
% similar to Assignment one to read encoder counts instead of the encoder.h
% library. 
%% Closed Loop Step Response
% 
k=0.033; %0.033 rad/sec
omega = 60;%1/volt
out = sim('controllerModel');
plot(out.controller);
title('Simulated Closed Loop Response');
ylabel('Angular position(rad)');
% *Disscussion: Simulated vs Experimental*
% My experimental closed loop response was more accurate than my simulated
% experimental response. This is because I was able to fine tune my kp and
% ki values within my controller arduino code to ensure that I was getting
% the most accurate response and rise time. Furthermore, the experimental
% system has no overshoot because of my ki tuning and because of friction
% and other constants not acounted for in the simulation. My simulated
% closed roop response and about an 11% overshoot and about a 0.75 second
% rise time.  My experimental rise time was much faster due to my kp and ki
% fine tuning. 

