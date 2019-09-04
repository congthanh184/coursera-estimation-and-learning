dt = 0.01;
T = 10;

%%
t = 1:dt:T;
actual_signal = 3*cos(t*1.5);
observe_noise = mvnrnd([-0.1, 0.15], [0.9 0; 0 0.5], numel(t))';
actual_x = [actual_signal; actual_signal] + observe_noise;

%%

A = eye(2);
C = eye(2);
Sigma_M = 0.005 .* eye(2);
R = 0.35 .* eye(2);
Pt = 0.03 .* eye(2);

x_hat = actual_x(:, 1);
D = [0.5 0.5];
y = D * actual_x(:, 1);
for i = 2:numel(t)
    % get the observe data
    z = actual_x(:, i);
    
    % estimate x_hat and P_hat
    P = A*Pt*A' + Sigma_M;
    
    K = P*C'/(R+C*P*C');
    x_hat(:, i) = A * x_hat(:, i-1) + K*(z - C*x_hat(:, i-1));
    y(i) = D * x_hat(:, i);
    Pt = P - K*C*P;
end
close all
plot(t, actual_signal, 'y-');
hold on
plot(t, y, 'r-');