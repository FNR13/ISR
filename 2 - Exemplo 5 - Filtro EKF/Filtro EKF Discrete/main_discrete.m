clc
clear

%% Inputs 
%-------------------------------------------------------------------------
% Real system
real_inital_position = [0.5;0.3];
real_vc = [0.1; 1]; % Current velocity to be estimated

use_r_noise = 1;
use_vw_noise = 1;
output_noise_power = 0.001;
vw_noise_power = 0.002;

% Trajectory
R_trajectory = 1;
yaw_rate = 0.2;
%--------------------------------------------------------------------------
% System initiation
    Pb = [0.5,1]; % Beacon Position   
    % Sampling Time
    Ts = 0.020;
%--------------------------------------------------------------------------
% Filter initiation
    % State Noise
    Q1 = 1; % Position
    Q2 = 1; % Current Noise
    
    % Measurement Noise
    R = 100000; % Range Noise

    % Initial estimate
    initial_x_estimate = 0;
    initial_y_estimate = 0;
    initial_vc_x = 0;
    initial_vc_y = 0;
    
    % Initial covariance matrix
    value = 10000;
    Position_variance = value;
    Current_velocity_variance = value;
%--------------------------------------------------------------------------

%% System
% System states are x = [position and current velocity],
% the inputs is the doppler velocity meausred u = [v_w]
% The output is the range of the vessel to a known position beacon
% 
% Position - p = (x, y)^T
% Velocity - v = (x_dot, y_dot)^T

% System Dynamics
I = eye(2);
O = zeros(2);

A = [O, I;
     O, O];

B = [I;
     O];
% Convert to discrete
sysc = ss(A,B,eye(4),0);
sysd = c2d(sysc,Ts,'Tustin');

F = sysd.A;
G = sysd.B;

% Nonlinear output matrix - h - range calculation
h = @(x,y) sqrt((x-Pb(1))^2 + (y-Pb(2))^2);

% jacobian of nonlinear output matrix 
J_h = @(x_hat,y_hat, v_cx_hat, v_cy_hat) ...
    [...
    (x_hat-Pb(1))/h(x_hat,y_hat), ...
    (y_hat-Pb(2))/h(x_hat,y_hat), ...
    0, ...
    0 ...
    ];


D = 0;

%% Filter Initiati

state0_hat = [initial_x_estimate; initial_y_estimate; initial_vc_x; initial_vc_y];

P0 = diag([Position_variance,Position_variance,Current_velocity_variance,Current_velocity_variance]);

Q = diag([Q1,Q1,Q2,Q2]);

%% Pass data to sim

sim_data = struct;
sim_data.Beacon_position = Pb;
sim_data.F = F;
sim_data.G = G;
sim_data.state_noise = Q;
sim_data.measurement_noise = R;
sim_data.Ts = Ts;

% Create a bus object
busInfo = Simulink.Bus.createObject(sim_data);
busObject = eval(busInfo.busName);

% run(simulation.slx)
sim("simulation_discrete",80);