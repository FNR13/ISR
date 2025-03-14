pm = [0;0];
previous_state_hat = [initial_x_estimate; initial_y_estimate; initial_surge_estimate; initial_yaw_estimate; initial_r_estimate];
previous_P = P0;

F_linear = F;
% Initialize variable from workspace
    % Known:

    % System dynamics
    % F_linear = sim_data.F;
    % H = sim_data.H;
    % Ts = sim_data.Ts;

    % Nonlinear dynamics matrix - position calculation
    f_x = @(x,u,yaw) x + u * cos(yaw) * Ts ;
    f_y = @(y,u,yaw) y + u * sin(yaw) * Ts ;
    
    % jacobian of nonlinear output matrix 
    J_f = @(x_hat,y_hat, u_hat, yaw_hat, r_hat) [...
        1, 0, cos(yaw_hat) * Ts, -u_hat * sin(yaw_hat) * Ts, 0; ...
        0, 1, sin(yaw_hat) * Ts, u_hat * cos(yaw_hat) * Ts, 0 ];

    % Filter initiation
    Q = sim_data.state_noise;
    R = sim_data.measurement_noise;

%% Extended Kalmen Filter Logic

x_previous = previous_state_hat(1);
y_previous = previous_state_hat(2);
u_previous = previous_state_hat(3);
yaw_previous = previous_state_hat(4);
r_previous = previous_state_hat(5);

% Linearize dynamics

F_linearized = J_f(x_previous, y_previous, u_previous, yaw_previous, r_previous);

F_full = [F_linearized; F_linear];

% Prediction
prediction_P = F_full * previous_P * F_full.'+ Q;

x_prediction = f_x(x_previous,u_previous,yaw_previous);
y_prediction = f_y(y_previous,u_previous,yaw_previous);

state_prediction = F_linear * [x_previous; y_previous; u_previous; yaw_previous; r_previous]; 
state_prediction = [x_prediction; y_prediction; state_prediction];

output_prediction = H * state_prediction;

% Update
K = prediction_P * H.' * (H * prediction_P * H.' + R)^(-1);

error = pm-output_prediction;
state_hat = state_prediction + K*error;

I = eye(size(state_prediction));
P = (I - K * H)* prediction_P;
