%% Demo 1: Control Design and Method
% My Demo 1 design process:
% 
% # Determine the angular velocity of each wheel 
% # Convert angular velocity to gain
% # Determine the transfer functions
% #Create PI controllers and find kp and ki values
% 

%% Determining wheel one trasnfer function
k1 = 0.04857657658; %rad/(volt*sec)
omega1 = 100; 
out = sim('transferFunctionWheel1');
plot(out.wheel1);
title('Simulated Open Loop Response for 1st wheel');
ylabel('Angular Velocity(rad/sec)');

%% Determining wheel two transfer function
k2 = 0.04885585586;
omega2 = 100; 
out = sim('transferFunctionWheel2');
plot(out.wheel2);
title('Simulated Open Loop Response for 2nd wheel');
ylabel('Angular Velocity(rad/sec)');

%% Controller for Wheel1
k1 = 0.04857657658; %rad/(volt*sec)
omega1 = 100;
out = sim('wheel1Controller');
plot(out.controller1);
title('Simulated Closed Loop Response: First Wheel');
ylabel('Angular position(rad)');
%P=615.397990431844
%I=309.498975578068

%% Controller for Wheel2
k2 = 0.04885585586;
omega2 = 100; 
out = sim('wheel2Controller');
plot(out.controller2);
title('Simulated Closed Loop Response: Second Wheel');
ylabel('Angular position(rad)');
%P=696.274989239527
%I=394.448534034899

%% Forward Velocity Transfer Function
% Determining the forward velocity
thetaRightWheel = 1.18;
thetaLeftWheel = 1.37;
v1 = .5; %voltage
v2 = .5; %voltage
r= 0.07485; %in meters, will have to change 
velocity = ((thetaRightWheel+thetaLeftWheel)*r)/2;
omega = 100;
out = sim('forwardTransferFunction');
plot(out.forwardPositionTransferFcn);

%% Forward Velocity Controller
% implementing P controller
thetaRightWheel = 1.18;
thetaLeftWheel = 1.37;
v1 = .5; %voltage
v2 = .5; %voltage
r= 0.07485; %in meters, will have to change 
velocity = ((thetaRightWheel+thetaLeftWheel)*r)/2;
omega = 100;
out = sim('forwardControl');
plot(out.forwardVelocity);
%P = 186623

%% Angular Velocity Transfer Function
% determing the angular velocity
thetaRightWheel = 1.18;
thetaLeftWheel = 1.37;
v1 = .5; %voltage
v2 = .5; %voltage
d = 0.29; %distance between wheels
r = 0.07485;
velocity = ((thetaRightWheel-thetaLeftWheel)/d)*r;
omega = 1000;
out = sim('angularTransferFunction');
plot(out.angularTF);

%% Angular Velocity Controller
% angular Velocity Controller
thetaRightWheel = 1.18;
thetaLeftWheel = 1.37;
v1 = .5; %voltage
v2 = .5; %voltage
d = 0.29; %distance between wheels
r = 0.07485;
velocity = ((thetaRightWheel-thetaLeftWheel)/d)*r;
omega = 1000;
out = sim('angularController');
plot(out.angularVelocity);
%P = -363178.639861224
