% pure_pursuit.m

% Time settings
dt = 0.01;
t_max = 20;
t = 0:dt:t_max;
N = length(t);

% Pursuer initial conditions
pursuer_pos = zeros(2, N);
pursuer_pos(:,1) = [0; 0];
pursuer_speed = 250; % m/s
pursuer_heading = deg2rad(45);

% Target initial conditions
target_pos = zeros(2, N);
target_pos(:,1) = [3000; 1000];
target_vel = [-150; 50];

% Simulation loop
for k = 1:N-1
    los_vector = target_pos(:,k) - pursuer_pos(:,k);
    pursuer_heading = atan2(los_vector(2), los_vector(1));

    pursuer_vel = pursuer_speed * [cos(pursuer_heading); sin(pursuer_heading)];
    pursuer_pos(:,k+1) = pursuer_pos(:,k) + pursuer_vel * dt;

    target_pos(:,k+1) = target_pos(:,k) + target_vel * dt;

    range = norm(los_vector);
    if range < 10
        fprintf('Intercept at t = %.2f s, range = %.2f m\n', t(k), range);
        break;
    end
end

% Plot trajectory
figure;
plot(pursuer_pos(1,1:k), pursuer_pos(2,1:k), 'b', 'LineWidth', 2);
hold on;
plot(target_pos(1,1:k), target_pos(2,1:k), 'r', 'LineWidth', 2);
plot(pursuer_pos(1,1), pursuer_pos(2,1), 'bo', 'MarkerSize', 8);
plot(target_pos(1,1), target_pos(2,1), 'ro', 'MarkerSize', 8);
legend('Pursuer (Pure Pursuit)', 'Target', 'Pursuer Start', 'Target Start');
grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)');
title('Pure Pursuit Intercept');