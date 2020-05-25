%% main.m
%
% Description:
%   Application entry point.
%
% Inputs: none
%
% Outputs: none
%
% Notes:

function main

%% Initialize environment
clear;
close all;
clc;

init_env();

%% Initialize parameters
params = init_params;

%% Visualize the robot in its initial state
x_IC = [params.sim.ICs.theta_bike;
        params.sim.ICs.theta_mw;
        params.sim.ICs.dtheta_bike;
        params.sim.ICs.dtheta_mw];

t_curr = 0;

% create a place for constraint forces populated in
% robot_dynamic_constraints function
F_calc = [];
tsim = [];
xsim = [];

% Set integration options - mainly events
options = odeset('Events',@robot_events);

while params.sim.tfinal - t_curr > params.sim.dt
        
    tspan_passive = t_curr:params.sim.dt:params.sim.tfinal;
    
    [tseg, xseg, ~, ~, ~] = ode45(@robot_dynamics_constraints, tspan_passive, x_IC', options);
    
    % extract info from the integration
    tsim = [tsim;tseg]; % build up the time vector after each event
    xsim = [xsim;xseg]; % build up the calculated state after each event
    
    t_curr = tsim(end); % set the current time to where the integration stopped
    x_IC = xsim(end,:); % set the initial condition to where the integration stopped
      
end

% transpose xsim_passive so that it is 5xN (N = number of timesteps):
 
 figure;
 
 xplot = xsim';
  
 % plot theta_bike and theta_mw of the robot
 subplot(2,1,1), plot(tsim,xplot(1,:),'b-',...
                      tsim,xplot(2,:),'r-','LineWidth',2);
                 legend('theta_bike','theta_mw');
                  
 % plot the angular velocity of the bike and MW of the robot
 subplot(2,1,2), plot(tsim,xplot(3,:),'b:',...
                      tsim,xplot(4,:),'r:','LineWidth',2);
                  legend('dtheta_bike','dtheta_mw');

 pause(1); % helps prevent animation from showing up on the wrong figure
 
% Let's resample the simulator output so we can animate with evenly-spaced
% points in (time,state).
% 1) deal with possible duplicate times in tsim:
% (https://www.mathworks.com/matlabcentral/answers/321603-how-do-i-interpolate-1d-data-if-i-do-not-have-unique-values
tsim = cumsum(ones(size(tsim)))*eps + tsim;

% 2) resample the duplicate-free time vector:
t_anim = 0:params.viz.dt:tsim(end);

% 3) resample the state-vs-time array:
% x_anim = interp1(tsim, xsim, t_anim); %x_anim doesn't run in airborne
x_anim = xsim'; % transpose so that xsim is 2xN (N = number of timesteps)
 
animate_robot(x_anim(1:2,:),params,'trace_cart_com',false,...
'trace_pend_com',false,'trace_pend_tip',false,'video',true);
 
 fprintf('Done passive simulation.\n');



function [dx] = robot_dynamics_constraints(t,x)
% Robot Dynamics
% Description:
%   Computes the constraint forces: 
%       Fnow = inv(A*Minv*A')*(A*Minv*(Q-H) + Adotqdot)
%
%   Also computes the derivative of the state:
%       x_dot(1:2) = (I - A'*inv(A*A')*A)*x(6:10)
%       x_dot(3:4) = inv(M)*(Q - H - A'F)
%
% Inputs:
%   t: time (scalar)
%   x: the 4x1 state vector
%   params: a struct with many elements, generated by calling init_params.m
%
% Outputs:
%   dx: derivative of state x with respect to time.
% for convenience, define q_dot
dx = zeros(numel(x),1);
nq = numel(x)/2;    % assume that x = [q;q_dot];
q_dot = x(nq+1:2*nq);

% tau = params.model.dyn.tau_mw * 0.05;
tau = params.model.dyn.tau_mw;

if t > 1
    tau = params.model.dyn.tau_mw;
end

Q = [0;tau];

% find the parts that don't depend on constraint forces
H = H_eom(x,params);
Minv = inv_mass_matrix(x,params);
[A_all,Hessian] = constraint_derivatives(x,params);

switch params.sim.constraints
    case 'balancing'
        disp("Changed Constraint!")
        %A = A_all(1,:);
        %Adotqdot = q_dot'*Hessian(:,:,1)*q_dot;  % robot position x-constraint
        %Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
        dx(1:nq) = eye(nq)*x(3:4);
        dx(nq+1:2*nq) = Minv*(Q - H);
        %F_calc = Fnow;
end
end
 
function [value,isterminal,direction] = robot_events(~,~)
    switch params.sim.constraints
        case ['balancing']
            value = 1;
            isterminal = 0;
            direction = 0;
    end
end 

% %% Control the unstable equilibrium with LQR
% A = upright_state_matrix(params);
% B = upright_input_matrix(params);
% 
% % numerical verify the rank of the controllability matrix:
% Co = [B, A*B, (A^2)*B, (A^3)*B];
% fprintf('rank(Co) = %d.\n',rank(Co));
% 
% % control design: weights Q and R:
% Q = diag([5000,100,1,1]);    % weight on regulation error
% R = 1;                  % weight on control effort
% 
% % compute and display optimal feedback gain matrix K:
% K = lqr(A,B,Q,R);
% buf = '';
% for i = 1:size(K,2)
%     buf = [buf,'%5.3f '];
% end
% buf = [buf,'\n'];
% fprintf('LQR: K = \n');
% fprintf(buf,K');
% 
% % we could ask what are the eigenvalues of the closed-loop system:
% eig(A - B*K)
% 
% % add K to our struct "params":
% params.control.inverted.K = K;
% 
% % Simulate the robot under this controller:
% tspan_stabilize = 0:params.sim.dt:5;
% [tsim_stabilize, xsim_stabilize] = ode45(@(t,x) robot_dynamics(...
%     t,x,0,params,'controller','stabilize'),...
%     tspan_stabilize, x_IC');
% 
% % tranpose xsim_passive so that it is 4xN (N = number of timesteps):
% xsim_stabilize = xsim_stabilize'; % required by animate_robot.m
% 
% figure;
% subplot(2,1,1), plot(tsim_stabilize,xsim_stabilize(1,:),'b-',...
%                      tsim_stabilize,xsim_stabilize(2,:),'r-','LineWidth',2);
% subplot(2,1,2), plot(tsim_stabilize,xsim_stabilize(3,:),'b:',...
%                      tsim_stabilize,xsim_stabilize(4,:),'r:','LineWidth',2);
% pause(1); % helps prevent animation from showing up on the wrong figure
% 
% 
% animate_robot(xsim_stabilize(1:2,:),params,'trace_cart_com',true,...
%     'trace_pend_com',true,'trace_pend_tip',true,'video',true);
% fprintf('Done passive simulation.\n');

end
