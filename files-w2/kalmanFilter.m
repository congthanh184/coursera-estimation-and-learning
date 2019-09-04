function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)
    dt = 0.0330;
    predictFrame = 1;
    deltaT = predictFrame * dt;
    
    A = [1 0 deltaT 0; 0 1 0 deltaT; 0 0 1 0; 0 0 0 1];
    C = [1 0 0 0; 0 1 0 0];
    R = 0.15 * eye(2);
    errM = diag([0.07 0.07 7 7]);    
%     param.P = diag([0.1, 0.1, 9, 9]);
    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = diag([3 3 10 10]);
        param.initSpeed = true;
%         param.P = diag([0.1, 0.1, 9, 9]);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    if (param.initSpeed == true)
        vx = (x - state(1)) / (t - previous_t);
        vy = (y - state(2)) / (t - previous_t);  
        state(3) = vx; state(4) = vy;
        param.initSpeed = false;
    end
    
    P = A * param.P * A' + errM;
    
    K = P * C' / (R + C * P * C');
    state = A * state' + K * ( [x; y] - C * A * state');
    param.P = P - K * C * P;
    % Predict 330ms into the future
    state = state';
%     predictx = x + vx * 0.330;
%     predicty = y + vy * 0.330;
    predictx = state(1) + state(3) * 0.330;
    predicty = state(2) + state(4) * 0.330;

    % State is a four dimensional element
%     state = [x, y, vx, vy];
    
%     predictx = state(1);
%     predicty = state(2);
end
