clc
clear

%% Inputs 
%-------------------------------------------------------------------------
% Real system
real_inital_position = [-0.4;0.3];
real_inital_surge = 2;
real_inital_yaw = 25;
real_inital_r = 6;

use_position_noise = 0;
output_position_power = 0.005;

%--------------------------------------------------------------------------
% System initiation,
    % Sampling Time
    Ts = 0.1;
%--------------------------------------------------------------------------
% Filter initiation
    % State Noise
    Q_pos = 10; % Position
    Q_surge = 10; %
    Q_yaw = 10;
    Q_r = 10;

    % Measurement Noise
    R = 1; % Range Noise

    % Initial covariance matrix
    covariance = 10;

    % Initial estimate
    initial_x_estimate = real_inital_position(1) - 0.2 * real_inital_position(1);
    initial_y_estimate = real_inital_position(2) + 0.1 * real_inital_position(2);
    initial_surge_estimate = real_inital_surge + 0.02 * real_inital_surge;
    initial_yaw_estimate = real_inital_yaw + real_inital_yaw*0.01;
    initial_yaw_estimate = 0;

    initial_r_estimate = real_inital_r;
    initial_r_estimate = real_inital_r + 0.05 * real_inital_r;
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
f_x = @(x,u,yaw) x + u * cosd(yaw) * Ts ;
f_y = @(y,u,yaw) y + u * sind(yaw) * Ts ;

% jacobian of nonlinear output matrix 
J_f = @(x_hat,y_hat, u_hat, yaw_hat, r_hat) [...
    1, 0, cosd(yaw_hat) * Ts, -u_hat * sind(yaw_hat) * Ts, 0; ...
    0, 1, sind(yaw_hat) * Ts, u_hat * cosd(yaw_hat) * Ts, 0 ];

B = 0;

H = [1 0 0 0 0;
     0 1 0 0 0];


D = 0;

%% Filter Initiati

state0_hat = [initial_x_estimate; initial_y_estimate; initial_surge_estimate; initial_yaw_estimate; initial_r_estimate];

P0 = diag([covariance,covariance,covariance,covariance,covariance]);

Q = diag([Q_pos,Q_pos,Q_surge,Q_yaw,Q_r]);

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
sim("simulation_discrete",10);