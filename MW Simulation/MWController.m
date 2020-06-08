function [control,eint,prevError] = MWController(state,int,prev)

% Create setpoint and convert state into useful number
angle = mod(state,2*pi);
set = pi/2;

p = 18;
i = 0;
d = 90;

% Calculate error terms
error = set-angle;
eint = int+error;

control = p * error + i * (eint) + d * (error-prev);

prevError = error;