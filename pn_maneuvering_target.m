% pn_maneuvering_target.m

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

initial_los = [3000; 1000] - [0; 0];
pursuer_heading = atan2(initial_los(2), initial_los(1));
pursuer_vel = pursuer_speed * [cos(pursuer_heading); sin(pursuer_heading)];

% Target initial conditions
target_pos = zeros(2, N);
target_pos(:,1) = [3000; 1000];
target_vel = [-150; 50];

% Maneuver settings
maneuver_start_time = 3.0;  % target starts turning at t = 3s
target_turn_rate = deg2rad(15); % rad/s, constant turn rate after maneuver starts

% Storage
accel_cmd = zeros(1, N);
miss_history = zeros(1, N);

% Simulation loop
for k = 1:N-1

    rel_pos = target_pos(:,k) - pursuer_pos(:,k);
    rel_vel = target_vel - pursuer_vel;

    range = norm(rel_pos);
    miss_history(k) = range;

    los_rate = (rel_pos(1)*rel_vel(2) - rel_pos(2)*rel_vel(1)) / (range^2);
    closing_vel = -dot(rel_pos, rel_vel) / range;

    % PN acceleration command
    a_cmd = Nav * closing_vel * los_rate;
    accel_cmd(k) = a_cmd;

    vel_dir = pursuer_vel / norm(pursuer_vel);
    perp_dir = [-vel_dir(2); vel_dir(1)];

    pursuer_vel = pursuer_vel + a_cmd * perp_dir * dt;
    pursuer_vel = pursuer_speed * pursuer_vel / norm(pursuer_vel);

    pursuer_pos(:,k+1) = pursuer_pos(:,k) + pursuer_vel * dt;

    % Target maneuver logic
    if t(k) >= maneuver_start_time
        % Rotate target velocity vector by turn_rate * dt
        theta = target_turn_rate * dt;
        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        target_vel = R * target_vel;
    end

    target_pos(:,k+1) = target_pos(:,k) + target_vel * dt;

    if range < 10
        fprintf('PN Intercept at t = %.2f s, range = %.2f m\n', t(k), range);
        break;
    end
end

if k == N-1
    fprintf('No intercept. Minimum miss distance = %.2f m\n', min(miss_history(1:k)));
end

% Plot trajectory
figure;
plot(pursuer_pos(1,1:k), pursuer_pos(2,1:k), 'b', 'LineWidth', 2);
hold on;
plot(target_pos(1,1:k), target_pos(2,1:k), 'r', 'LineWidth', 2);
plot(pursuer_pos(1,1), pursuer_pos(2,1), 'bo', 'MarkerSize', 8);
plot(target_pos(1,1), target_pos(2,1), 'ro', 'MarkerSize', 8);
legend('Pursuer (PN)', 'Target (maneuvering)', 'Pursuer Start', 'Target Start');
grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)');
title('PN Intercept vs Maneuvering Target');

% Plot acceleration command
figure;
plot(t(1:k), accel_cmd(1:k), 'LineWidth', 1.5);
grid on;
xlabel('time (s)');
ylabel('Commanded lateral acceleration (m/s^2)');
title('PN Acceleration Command - Maneuvering Target');