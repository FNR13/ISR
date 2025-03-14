clc 
clear

%% System Dynamics
% System states are X = [yaw and yaw rate bias],
% the inputs is the gyroscope velocity meausred u = [r]
% The output is the yaw meausured

A = [0 1; 
     0 0];

B = [1;
     0];

C = [1 0];

D = 0;

yaw_0 = pi;

use_noise = 0;
yaw_noise = 0.2;
r_noise = 0.001;

% Bias to be estimated
bias = 1;

% Kalmen gains
state_noise_1 = 1;
state_noise_2 = 1;

measurement_noise_1 = 1;

% From kalmen logic
k1 = 2*sqrt(state_noise_2) + state_noise_1/(sqrt(measurement_noise_1));
k2 = state_noise_2/sqrt(measurement_noise_1);

% run('EstimationSimulations.slx')
sim('EstimationSimulations',30);