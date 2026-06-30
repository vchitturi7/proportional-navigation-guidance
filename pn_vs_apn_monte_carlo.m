% pn_vs_apn_monte_carlo.m

% Fixed simulation settings
dt = 0.01;
t_max = 20;
pursuer_speed = 250;
Nav = 4;
n_trials = 200;

% Storage for both guidance laws
miss_distances_PN = zeros(1, n_trials);
miss_distances_APN = zeros(1, n_trials);
hit_flags_PN = false(1, n_trials);
hit_flags_APN = false(1, n_trials);
maneuver_start_times = zeros(1, n_trials);
target_turn_rates = zeros(1, n_trials);

rng(42); % same seed as before for identical scenarios

for trial = 1:n_trials

    maneuver_start_time = 1 + 4*rand();
    target_turn_rate = deg2rad(5 + 25*rand());

    maneuver_start_times(trial) = maneuver_start_time;
    target_turn_rates(trial) = rad2deg(target_turn_rate);

    % Run both guidance laws on the identical scenario
    [miss_PN, hit_PN] = run_intercept(maneuver_start_time, target_turn_rate, ...
        dt, t_max, pursuer_speed, Nav, false);
    [miss_APN, hit_APN] = run_intercept(maneuver_start_time, target_turn_rate, ...
        dt, t_max, pursuer_speed, Nav, true);

    miss_distances_PN(trial) = miss_PN;
    miss_distances_APN(trial) = miss_APN;
    hit_flags_PN(trial) = hit_PN;
    hit_flags_APN(trial) = hit_APN;

end

% Results summary
fprintf('\n--- PN vs APN Comparison (N = %d trials) ---\n', n_trials);
fprintf('PN hit rate:  %.1f%%   Mean miss: %.2f m\n', ...
    sum(hit_flags_PN)/n_trials*100, mean(miss_distances_PN));
fprintf('APN hit rate: %.1f%%   Mean miss: %.2f m\n', ...
    sum(hit_flags_APN)/n_trials*100, mean(miss_distances_APN));

% Check specifically on the previously identified PN failure cases
fprintf('\n--- Previously Failed PN Cases, Re-tested with APN ---\n');
failure_idx = miss_distances_PN > 50;
fprintf('%-20s %-20s %-15s %-15s\n', 'Maneuver Start (s)', 'Turn Rate (deg/s)', 'PN Miss (m)', 'APN Miss (m)');
for i = find(failure_idx)
    fprintf('%-20.2f %-20.2f %-15.2f %-15.2f\n', ...
        maneuver_start_times(i), target_turn_rates(i), ...
        miss_distances_PN(i), miss_distances_APN(i));
end

% Plot comparison scatter
figure;
scatter(target_turn_rates, miss_distances_PN, 40, 'r', 'filled', 'DisplayName', 'PN');
hold on;
scatter(target_turn_rates, miss_distances_APN, 40, 'b', 'filled', 'DisplayName', 'APN');
yline(50, '--k', 'Failure threshold');
xlabel('Target turn rate (deg/s)');
ylabel('Miss distance (m)');
title('PN vs APN Miss Distance Across Turn Rate');
legend('Location', 'best');
grid on;


function [miss_dist, hit] = run_intercept(maneuver_start_time, target_turn_rate, ...
    dt, t_max, pursuer_speed, Nav, use_apn)

    t = 0:dt:t_max;
    N = length(t);

    pursuer_pos = zeros(2, N);
    pursuer_pos(:,1) = [0; 0];

    initial_los = [3000; 1000] - [0; 0];
    pursuer_heading = atan2(initial_los(2), initial_los(1));
    pursuer_vel = pursuer_speed * [cos(pursuer_heading); sin(pursuer_heading)];

    target_pos = zeros(2, N);
    target_pos(:,1) = [3000; 1000];
    target_vel = [-150; 50];

    miss_history = inf(1, N);
    hit = false;

    for k = 1:N-1
        rel_pos = target_pos(:,k) - pursuer_pos(:,k);
        rel_vel = target_vel - pursuer_vel;

        range = norm(rel_pos);
        miss_history(k) = range;

        los_rate = (rel_pos(1)*rel_vel(2) - rel_pos(2)*rel_vel(1)) / (range^2);
        closing_vel = -dot(rel_pos, rel_vel) / range;

        a_cmd = Nav * closing_vel * los_rate;

        % APN adds target acceleration term
        if use_apn && t(k) >= maneuver_start_time
            % Target acceleration magnitude from circular turn: a = v * turn_rate
            target_speed = norm(target_vel);
            target_accel_mag = target_speed * target_turn_rate;

            % Direction of target acceleration (perpendicular to its velocity)
            target_vel_dir = target_vel / target_speed;
            target_accel_dir = [-target_vel_dir(2); target_vel_dir(1)];
            target_accel = target_accel_mag * target_accel_dir;

            % Project onto perpendicular-to-LOS direction
            los_dir = rel_pos / range;
            los_perp = [-los_dir(2); los_dir(1)];
            a_target_perp = dot(target_accel, los_perp);

            a_cmd = a_cmd + (Nav/2) * a_target_perp;
        end

        vel_dir = pursuer_vel / norm(pursuer_vel);
        perp_dir = [-vel_dir(2); vel_dir(1)];

        pursuer_vel = pursuer_vel + a_cmd * perp_dir * dt;
        pursuer_vel = pursuer_speed * pursuer_vel / norm(pursuer_vel);

        pursuer_pos(:,k+1) = pursuer_pos(:,k) + pursuer_vel * dt;

        if t(k) >= maneuver_start_time
            theta = target_turn_rate * dt;
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            target_vel = R * target_vel;
        end

        target_pos(:,k+1) = target_pos(:,k) + target_vel * dt;

        if range < 10
            hit = true;
            break;
        end

        if k > 100 && range > 5000
            break;
        end
    end

    miss_dist = min(miss_history(1:k));

end