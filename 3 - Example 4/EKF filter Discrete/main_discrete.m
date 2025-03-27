clc
clear

%% Inputs 
%-------------------------------------------------------------------------
% Real system
real_inital_position = [-0.4;0.3];
real_inital_surge = 5;
real_inital_yaw = pi/2;
real_inital_r = -0.2;

use_position_noise = 1;
output_position_power = 0.005;

%--------------------------------------------------------------------------
% System initiation,
    % Sampling Time
    Ts = 0.1;
%--------------------------------------------------------------------------
% Filter initiation
    % State Noise
    Q_pos = 1; % Position
    Q_surge = 0.1; %
    Q_yaw = pi/16; % 11.25 degrees
    Q_r = 0.01;

    % Measurement Noise
    R_pos = 10; % Range Noise
   
    % Initial estimate
    initial_x_estimate = real_inital_position(1) - rand * real_inital_position(1);
    initial_y_estimate = real_inital_position(2) + rand * real_inital_position(2);
    initial_surge_estimate = real_inital_surge + (rand*0.1) * real_inital_surge;
    % initial_surge_estimate = 0;
    initial_yaw_estimate = real_inital_yaw + (rand*0.2) * real_inital_yaw;
    initial_yaw_estimate = 0;
    initial_r_estimate = real_inital_r + (rand*0.01) * real_inital_r;
    initial_r_estimate = 0;
%--------------------------------------------------------------------------

%% System
% System states are x = [position, surge, yaw and yaw rate(r)],
% There are no inputs
% The output is the position of the vessel
% 
% Position - p = (x, y)^T

% System Dynamics
F = [
    0 0 1 0 0;
    0 0 0 1 Ts;
    0 0 0 0 1];

% Nonlinear dynamics matrix - position calculation
f_x = @(x,u,yaw) x + u * cos(yaw) * Ts ;
f_y = @(y,u,yaw) y + u * sin(yaw) * Ts ;

% jacobian of nonlinear output matrix 
J_f = @(x_hat,y_hat, u_hat, yaw_hat, r_hat) [...
    1, 0, cos(yaw_hat) * Ts, -u_hat * sin(yaw_hat) * Ts, 0; ...
    0, 1, sin(yaw_hat) * Ts, u_hat * cos(yaw_hat) * Ts, 0 ];

B = 0;

H = [1 0 0 0 0;
     0 1 0 0 0];


D = 0;

%% Filter Initiatiom

state0_hat = [initial_x_estimate; initial_y_estimate; initial_surge_estimate; initial_yaw_estimate; initial_r_estimate];

Q = diag([Q_pos,Q_pos,Q_surge,Q_yaw,Q_r]);

% P0 = diag([covariance,covariance,covariance,covariance,covariance]);
P0 = 2*Q;
R = diag([R_pos,R_pos]);

%% Pass data to sim

sim_data = struct;
sim_data.F = F;
sim_data.H = H;
sim_data.state_noise = Q;
sim_data.measurement_noise = R;
sim_data.Ts = Ts;

% Create a bus object
busInfo = Simulink.Bus.createObject(sim_data);
busObject = eval(busInfo.busName);

% run(simulation.slx)
sim("simulation_discrete",30);