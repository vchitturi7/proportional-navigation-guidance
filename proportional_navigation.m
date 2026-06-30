% proportional_navigation.m

% Time settings
dt = 0.01;
t_max = 20;
t = 0:dt:t_max;
N = length(t);

% Navigation constant
Nav = 4;

% Pursuer initial conditions
pursuer_pos = zeros(2, N);
pursuer_pos(:,1) = [0; 0];
pursuer_speed = 250; % m/s

% Initial pursuer velocity direction (roughly toward target)
initial_los = [3000; 1000] - [0; 0];
pursuer_heading = atan2(initial_los(2), initial_los(1));
pursuer_vel = pursuer_speed * [cos(pursuer_heading); sin(pursuer_heading)];

% Target initial conditions
target_pos = zeros(2, N);
target_pos(:,1) = [3000; 1000];
target_vel = [-150; 50];

% Storage for LOS angle and acceleration command
los_angle = zeros(1, N);
los_angle(1) = atan2(initial_los(2), initial_los(1));
accel_cmd = zeros(1, N);

% Simulation loop
for k = 1:N-1

    % Relative position and velocity
    rel_pos = target_pos(:,k) - pursuer_pos(:,k);
    rel_vel = target_vel - pursuer_vel;

    range = norm(rel_pos);

    % Current LOS angle
    los_angle(k) = atan2(rel_pos(2), rel_pos(1));

    % LOS rate using the standard PN formula:
    % lambda_dot = (rel_pos x rel_vel) / |rel_pos|^2
    % This is the 2D cross product (scalar)
    los_rate = (rel_pos(1)*rel_vel(2) - rel_pos(2)*rel_vel(1)) / (range^2);

    % Closing velocity = negative rate of change of range
    closing_vel = -dot(rel_pos, rel_vel) / range;

    % PN acceleration command (perpendicular to current pursuer heading)
    a_cmd = Nav * closing_vel * los_rate;
    accel_cmd(k) = a_cmd;

    % Apply acceleration perpendicular to velocity direction
    vel_dir = pursuer_vel / norm(pursuer_vel);
    perp_dir = [-vel_dir(2); vel_dir(1)]; % 90 degree rotation

    pursuer_vel = pursuer_vel + a_cmd * perp_dir * dt;

    % Maintain constant speed (renormalize)
    pursuer_vel = pursuer_speed * pursuer_vel / norm(pursuer_vel);

    % Update positions
    pursuer_pos(:,k+1) = pursuer_pos(:,k) + pursuer_vel * dt;
    target_pos(:,k+1) = target_pos(:,k) + target_vel * dt;

    % Check for intercept
    if range < 10
        fprintf('PN Intercept at t = %.2f s, range = %.2f m\n', t(k), range);
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
legend('Pursuer (PN)', 'Target', 'Pursuer Start', 'Target Start');
grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)');
title('Proportional Navigation Intercept');

% Plot acceleration command over time
figure;
plot(t(1:k), accel_cmd(1:k), 'LineWidth', 1.5);
grid on;
xlabel('time (s)');
ylabel('Commanded lateral acceleration (m/s^2)');
title('PN Acceleration Command Over Time');