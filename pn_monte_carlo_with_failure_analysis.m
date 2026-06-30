% pn_monte_carlo_with_failure_analysis.m

% Fixed simulation settings
dt = 0.01;
t_max = 20;
pursuer_speed = 250;
Nav = 4;
n_trials = 200;

% Storage for results
miss_distances = zeros(1, n_trials);
intercept_times = NaN(1, n_trials);
hit_flags = false(1, n_trials);
maneuver_start_times = zeros(1, n_trials);
target_turn_rates = zeros(1, n_trials);

rng(42); % reproducible random seed

for trial = 1:n_trials

    % Randomize target maneuver parameters per trial
    maneuver_start_time = 1 + 4*rand();
    target_turn_rate = deg2rad(5 + 25*rand());

    % Store parameters for later analysis
    maneuver_start_times(trial) = maneuver_start_time;
    target_turn_rates(trial) = rad2deg(target_turn_rate);

    % Initial conditions
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

    for k = 1:N-1
        rel_pos = target_pos(:,k) - pursuer_pos(:,k);
        rel_vel = target_vel - pursuer_vel;

        range = norm(rel_pos);
        miss_history(k) = range;

        los_rate = (rel_pos(1)*rel_vel(2) - rel_pos(2)*rel_vel(1)) / (range^2);
        closing_vel = -dot(rel_pos, rel_vel) / range;

        a_cmd = Nav * closing_vel * los_rate;

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
            intercept_times(trial) = t(k);
            hit_flags(trial) = true;
            break;
        end

        if k > 100 && range > 5000
            break;
        end
    end

    miss_distances(trial) = min(miss_history(1:k));

end

% Results summary
hit_rate = sum(hit_flags) / n_trials * 100;
mean_miss = mean(miss_distances);
mean_intercept_time = mean(intercept_times(hit_flags));

fprintf('\n--- Monte Carlo Results (N = %d trials) ---\n', n_trials);
fprintf('Hit rate: %.1f%%\n', hit_rate);
fprintf('Mean miss distance: %.2f m\n', mean_miss);
fprintf('Mean intercept time (hits only): %.2f s\n', mean_intercept_time);

% Identify failures
miss_threshold = 50; % anything above 50m miss counts as a failure
failure_idx = miss_distances > miss_threshold;

fprintf('\nFailure cases (miss > %d m): %d out of %d\n', ...
    miss_threshold, sum(failure_idx), n_trials);

if sum(failure_idx) > 0
    fprintf('\nFailure conditions:\n');
    fprintf('%-20s %-20s %-15s\n', 'Maneuver Start (s)', 'Turn Rate (deg/s)', 'Miss (m)');
    for i = find(failure_idx)
        fprintf('%-20.2f %-20.2f %-15.2f\n', ...
            maneuver_start_times(i), target_turn_rates(i), miss_distances(i));
    end
end

% Plot 1 — miss distance histogram
figure;
histogram(miss_distances, 30);
xlabel('Miss distance (m)');
ylabel('Number of trials');
title(sprintf('Miss Distance Distribution (Nav = %d, %d trials)', Nav, n_trials));
grid on;

% Plot 2 — failure scatter: maneuver start vs turn rate, colored by outcome
figure;
scatter(maneuver_start_times(~failure_idx), target_turn_rates(~failure_idx), ...
    40, 'g', 'filled', 'DisplayName', 'Success');
hold on;
scatter(maneuver_start_times(failure_idx), target_turn_rates(failure_idx), ...
    80, 'r', 'filled', 'DisplayName', 'Failure (miss > 50m)');
xlabel('Maneuver start time (s)');
ylabel('Target turn rate (deg/s)');
title('PN Performance Across Maneuver Parameter Space');
legend('Location', 'best');
grid on;