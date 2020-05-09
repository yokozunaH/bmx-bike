%% init_params.m
%
% Description:
%   Initializes the values of many parameters, such as parameters in the
%   system dynamics, parameters that relate to simulating the system
%   forward in time, and parametes that relate to visualization/animation.
%
% Inputs:
%   none

% Outputs:
%   params: a struct with many elements

function params = init_params
    % parameters that appear in the dynamics:
     params.model.dyn.com.m = 5;    % mass of the bike body
     params.model.dyn.bw.m = 0.185;    % mass of bw
     params.model.dyn.fw.m = 0.185;    % mass of fw

     params.model.dyn.bw.I = 0.000380; % moment of inertia of bw
     params.model.dyn.fw.I = 0.000380; % moment of inertia of fw
     params.model.dyn.com.I = 0.1225; % moment of inertia of com

     params.model.dyn.g = 9.81;      % acceleration due to gravity
     params.model.dyn.tau_bw = -2;    % applied torque from the backwheel

    % parameters that help with visualizing the robot:
    params.model.geom.body.w = 0.3; % width of the bike body
    params.model.geom.body.h = 0.3; % height of the bike body
    params.model.geom.leg.l = 0.12;   % length of the bike legs
    params.model.geom.leg.w = 0.05; % width of the bike legs
    params.model.geom.leg.wheel_d = 0.05; % distance of leg above of wheel center

    params.model.geom.wheel.r = 0.0625; % radius of wheels

    params.model.geom.bw_fw.l = 0.34477; % distance between the two wheels
    
    params.model.geom.bw_com.l = 0.22757; % straight line distance between back wheel and CoM
    params.model.geom.bw_com.theta = 1.101; %Initial angle between back wheel and CoM

%     params.model.geom.fw_com.l = 0.31584;% straight line distance between back wheel and CoM
%     params.model.geom.fw_com.theta = 2.464; %Initial angle between back wheel and CoM

    params.viz.colors.body = [0.5 0.5 0.5];
    params.viz.colors.leg = [0.25 0.25 0.25];
    params.viz.colors.wheels = [0.75 0.75 0.75];
    params.viz.colors.tracers.body = 'r';
    params.viz.colors.tracers.leg = 'g';
    params.viz.colors.tracers.wheels = 'b';
    params.viz.axis_lims = [-0.5,3,-0.1,1];
    params.viz.dt = 0.05;

    % parameters related to simulating (integrating) the dynamics forward
    % in time:
    params.sim.ICs.x_bf = 0;      % initial positions
    params.sim.ICs.y_bf = params.model.geom.wheel.r;
    params.sim.ICs.theta_com = 0;
    params.sim.ICs.theta_fw = 0;
    params.sim.ICs.theta_bw = 0;
    params.sim.ICs.dx_com = 0;     % initial velocities
    params.sim.ICs.dy_com = 0;
    params.sim.ICs.dtheta_com = 0;
    params.sim.ICs.dtheta_fw = 0;
    params.sim.ICs.dtheta_bw = 0;
    params.sim.tfinal = 5;          % Length of the simulation
    params.sim.dt = 0.05;           % simulation timestep
    
    % variables related to the constraints
    params.sim.constraints.number = 4;  % total number of constraint equations
    
    % list of ground constraints True if wheel is on the ground; False otherwise
    params.sim.constraints = ['true','true']; % [back wheel, front wheel]
end
