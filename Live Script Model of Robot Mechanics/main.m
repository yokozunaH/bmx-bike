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
x_IC = [params.sim.ICs.x_com;
     params.sim.ICs.y_com;
     params.sim.ICs.theta_com;
     params.sim.ICs.theta_bw;
     params.sim.ICs.theta_fw;
     params.sim.ICs.dx_com;
     params.sim.ICs.dy_com;
     params.sim.ICs.dtheta_com;
     params.sim.ICs.dtheta_bw;
     params.sim.ICs.dtheta_fw];
 
plot_robot(x_IC(1:5),params,'new_fig',false);

% %% Simulate the robot forward in time with no control input
 tspan_passive = 0:params.sim.dt:5;
 [tsim_passive, xsim_passive] = ode45(@(t,x) robot_dynamics_constraints(...
     t,x,params), ...%,'controller','passive'),...
     tspan_passive, x_IC');
% 
% % tranpose xsim_passive so that it is 4xN (N = number of timesteps):
 xsim_passive = xsim_passive'; % required by animate_robot.m
% 
 figure;
 subplot(2,1,1), plot(tsim_passive,xsim_passive(2,:),'b-') %,...
                      %tsim_passive,xsim_passive(2,:),'r-','LineWidth',2);
 %subplot(2,1,2), plot(tsim_passive,xsim_passive(4,:),'b:',...
 %                     tsim_passive,xsim_passive(5,:),'r:','LineWidth',2);
% 
% 
 pause(1); % helps prevent animation from showing up on the wrong figure
 animate_robot(xsim_passive(1:5,:),params,'trace_cart_com',false,...
     'trace_pend_com',false,'trace_pend_tip',false,'video',true);
 fprintf('Done passive simulation.\n');
% 
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
