%% Demo 1: Control Design and Method
% My Demo 1 design process:
% 
% # Determine the angular velocity of each wheel 
% # ITEM2
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
