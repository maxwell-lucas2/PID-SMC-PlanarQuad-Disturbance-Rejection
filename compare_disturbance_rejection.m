%% PID vs SMC Disturbance Rejection Comparison
%  Maxwell Lucas — University of Vermont, Honors College Thesis
%  Runs both controllers on identical quad + disturbances, overlays results
clear; close all; clc;

%% Sim paramaters
dt       = 0.002;       % Simulation timestep [s]
ctrl_div = 5;           % Controller updates every ctrl_div steps
Tc       = dt * ctrl_div;  % = 0.01 s controller period
T_total  = 20.0;        % Duration [s]
t        = 0:dt:T_total;
N        = length(t);

x_cmd = 1.0;            % Commanded position [m]
z_cmd = 1.0;

% Disturbance scenarios
scenarios = {"Horizontal Wind", "Vertical Wind", "Attitude Torque"};
n_scenarios = length(scenarios);

% Fixed seed for same disturbances
seed = 12345;

% Colors
clr_pid = [0.00 0.45 0.74];   % Blue
clr_smc = [0.85 0.33 0.10];   % Orange

%% Run scenarios
for sc = 1:n_scenarios
    fprintf('--------- \n Scenario %d: %s ---------\n', sc, scenarios{sc});

    % Disturbances
    % Wind: [1.0, 3.0] m/s
    % Torque: [0.05, 0.2] N·m
    rng(seed);
    switch sc
        case 1  % Horizontal wind
            wind_x = RandomWindPulses(t, 3, [1.0 3.0], [0.5 2]);
            wind_z  = zeros(N,1);
            tau_dist = zeros(N,1);
        case 2  % Vertical wind
            wind_x   = zeros(N,1);
            wind_z   = RandomWindPulses(t, 3, [1.0 3.0], [0.5 2]);
            tau_dist = zeros(N,1);
        case 3  % Attitude torque
            wind_x   = zeros(N,1);
            wind_z   = zeros(N,1);
            tau_dist = RandomWindPulses(t, 3, [0.05 0.07], [0.3 1.0]);
    end
    wind_all = [wind_x, wind_z, tau_dist];  % Nx3

    % Run PID
    [pid_states, pid_F1, pid_F2, pid_log] = run_sim( ...
        'PID', x_cmd, z_cmd, t, dt, ctrl_div, wind_all);

    % Run SMC
    [smc_states, smc_F1, smc_F2, smc_log] = run_sim( ...
        'SMC', x_cmd, z_cmd, t, dt, ctrl_div, wind_all);

    % Compute metrics
    pid_m = compute_metrics(pid_states, pid_F1, pid_F2, x_cmd, z_cmd, t, dt);
    smc_m = compute_metrics(smc_states, smc_F1, smc_F2, x_cmd, z_cmd, t, dt);

    % Print comparison table
    fprintf('\n%-30s | %12s | %12s\n', 'Metric', 'PID', 'SMC');
    fprintf('%s\n', repmat('-', 1, 60));
    fprintf('%-30s | %12.4f | %12.4f\n', 'X RMSE [m]',         pid_m.rmse_x,    smc_m.rmse_x);
    fprintf('%-30s | %12.4f | %12.4f\n', 'Z RMSE [m]',         pid_m.rmse_z,    smc_m.rmse_z);
    fprintf('%-30s | %12.4f | %12.4f\n', 'X max error [m]',    pid_m.max_err_x, smc_m.max_err_x);
    fprintf('%-30s | %12.4f | %12.4f\n', 'Z max error [m]',    pid_m.max_err_z, smc_m.max_err_z);
    fprintf('%-30s | %12.4f | %12.4f\n', 'X mean |error| [m]', pid_m.mae_x,     smc_m.mae_x);
    fprintf('%-30s | %12.4f | %12.4f\n', 'Z mean |error| [m]', pid_m.mae_z,     smc_m.mae_z);
    fprintf('%-30s | %12.2f | %12.2f\n', 'Total effort [N*s]', pid_m.effort,    smc_m.effort);
    fprintf('%-30s | %12.2f | %12.2f\n', 'Thrust TV [N]',      pid_m.thrust_TV, smc_m.thrust_TV);
    fprintf('%-30s | %12.2f | %12.2f\n', 'Max |pitch| [deg]',  pid_m.max_pitch, smc_m.max_pitch);
    fprintf('%-30s | %12.4f | %12.4f\n', 'Settling time X [s]',pid_m.settle_x,  smc_m.settle_x);
    fprintf('%-30s | %12.4f | %12.4f\n', 'Settling time Z [s]',pid_m.settle_z,  smc_m.settle_z);

    % Extract state histories
    px  = pid_states(:,1);  pz  = pid_states(:,3);  pth  = pid_states(:,5);
    sx  = smc_states(:,1);  sz  = smc_states(:,3);  sth  = smc_states(:,5);

    % Position tracking overlay
    figure('Position', [50 50 1300 700], ...
           'Name', sprintf('Scenario %d: %s — Tracking', sc, scenarios{sc}));

    subplot(2,2,1);
    plot(t, px, '-', 'Color', clr_pid, 'LineWidth', 1.5); hold on;
    plot(t, sx, '-', 'Color', clr_smc, 'LineWidth', 1.5);
    yline(x_cmd, 'k--', 'LineWidth', 1);
    grid on; xlabel('Time [s]'); ylabel('X [m]');
    title('Horizontal Position'); legend('PID','SMC','Cmd','Location','best');

    subplot(2,2,2);
    plot(t, pz, '-', 'Color', clr_pid, 'LineWidth', 1.5); hold on;
    plot(t, sz, '-', 'Color', clr_smc, 'LineWidth', 1.5);
    yline(z_cmd, 'k--', 'LineWidth', 1);
    grid on; xlabel('Time [s]'); ylabel('Z [m]');
    title('Vertical Position'); legend('PID','SMC','Cmd','Location','best');

    subplot(2,2,3);
    plot(t, x_cmd - px, '-', 'Color', clr_pid, 'LineWidth', 1.2); hold on;
    plot(t, x_cmd - sx, '-', 'Color', clr_smc, 'LineWidth', 1.2);
    yline(0,'k--'); grid on; xlabel('Time [s]'); ylabel('Error [m]');
    title('X Tracking Error'); legend('PID','SMC','Location','best');

    subplot(2,2,4);
    plot(t, z_cmd - pz, '-', 'Color', clr_pid, 'LineWidth', 1.2); hold on;
    plot(t, z_cmd - sz, '-', 'Color', clr_smc, 'LineWidth', 1.2);
    yline(0,'k--'); grid on; xlabel('Time [s]'); ylabel('Error [m]');
    title('Z Tracking Error'); legend('PID','SMC','Location','best');

    sgtitle(sprintf('Scenario %d: %s — Position Tracking', sc, scenarios{sc}), ...
        'FontSize', 14, 'FontWeight', 'bold');

    % Pitch, thrusts, disturbance
    figure('Position', [100 100 1300 700], ...
           'Name', sprintf('Scenario %d: %s — Control', sc, scenarios{sc}));

    subplot(2,2,1);
    plot(t, rad2deg(pth), '-', 'Color', clr_pid, 'LineWidth', 1.2); hold on;
    plot(t, rad2deg(sth), '-', 'Color', clr_smc, 'LineWidth', 1.2);
    yline(0,'k--'); grid on; xlabel('Time [s]'); ylabel('Pitch [deg]');
    title('Pitch Angle'); legend('PID','SMC','Location','best');

    subplot(2,2,2);
    plot(t, pid_F1 + pid_F2, '-', 'Color', clr_pid, 'LineWidth', 1.0); hold on;
    plot(t, smc_F1 + smc_F2, '-', 'Color', clr_smc, 'LineWidth', 1.0);
    grid on; xlabel('Time [s]'); ylabel('Total Thrust [N]');
    title('Total Thrust'); legend('PID','SMC','Location','best');

    subplot(2,2,3);
    switch sc
        case 1
            plot(t, wind_x, 'r-', 'LineWidth', 1.2);
            ylabel('Wind X [m/s]'); title('Horizontal Wind Disturbance');
        case 2
            plot(t, wind_z, 'r-', 'LineWidth', 1.2);
            ylabel('Wind Z [m/s]'); title('Vertical Wind Disturbance');
        case 3
            plot(t, tau_dist, 'r-', 'LineWidth', 1.2);
            ylabel('\tau_{ext} [N\cdotm]'); title('External Torque Disturbance');
    end
    grid on; xlabel('Time [s]');

    subplot(2,2,4);
    % Differential thrust
    plot(t, pid_F1 - pid_F2, '-', 'Color', clr_pid, 'LineWidth', 1.0); hold on;
    plot(t, smc_F1 - smc_F2, '-', 'Color', clr_smc, 'LineWidth', 1.0);
    yline(0,'k--'); grid on; xlabel('Time [s]'); ylabel('F_1 - F_2 [N]');
    title('Differential Thrust (Torque Proxy)');
    legend('PID','SMC','Location','best');

    sgtitle(sprintf('Scenario %d: %s — Control Effort & Disturbance', ...
        sc, scenarios{sc}), 'FontSize', 14, 'FontWeight', 'bold');

    % SMC Sliding Surfaces
    figure('Position', [150 150 1000 350], ...
           'Name', sprintf('Scenario %d: %s — Sliding Surfaces', sc, scenarios{sc}));

    subplot(1,3,1);
    plot(t, smc_log.s_alt, 'm-', 'LineWidth', 1.2);
    yline(0,'k--'); grid on; xlabel('Time [s]'); ylabel('s_z');
    title('Altitude');

    subplot(1,3,2);
    plot(t, smc_log.s_lat, 'm-', 'LineWidth', 1.2);
    yline(0,'k--'); grid on; xlabel('Time [s]'); ylabel('s_x');
    title('Lateral');

    subplot(1,3,3);
    plot(t, smc_log.s_att, 'm-', 'LineWidth', 1.2);
    yline(0,'k--'); grid on; xlabel('Time [s]'); ylabel('s_\theta');
    title('Attitude');

    sgtitle(sprintf('Scenario %d: %s — SMC Sliding Surfaces', ...
        sc, scenarios{sc}), 'FontSize', 14, 'FontWeight', 'bold');
end

fprintf('\n===== All scenarios complete =====\n');

%% Simulation function
function [states, F1_log, F2_log, ctrl_log] = run_sim(ctrl_type, x_cmd, z_cmd, t, dt, ctrl_div, wind_all)

    N = length(t);

    % Create quad
    quad = basicQuad();
    quad.state = [x_cmd; 0; z_cmd; 0; 0; 0];  % Start at target

    % Create controller
    switch ctrl_type
        case 'PID'
            ctrl = PIDCtrl(quad);
        case 'SMC'
            ctrl = SMCCtrl(quad);
    end

    % Pre-allocate
    states = zeros(N, 6);
    F1_log = zeros(N, 1);
    F2_log = zeros(N, 1);
    ctrl_log.s_alt = zeros(N, 1);
    ctrl_log.s_lat = zeros(N, 1);
    ctrl_log.s_att = zeros(N, 1);
    ctrl_log.theta_cmd = zeros(N, 1);

    state = quad.state;
    states(1,:) = state';

    F1 = 0;  F2 = 0;
    update_ctr = ctrl_div;  % Trigger on first step

    for k = 1:N-1
        % Controller update at Tc rate
        if update_ctr >= ctrl_div
            [F1, F2, lg] = ctrl.update(x_cmd, z_cmd, state);
            ctrl_log.s_alt(k)     = lg.s_alt;
            ctrl_log.s_lat(k)     = lg.s_lat;
            ctrl_log.s_att(k)     = lg.s_att;
            ctrl_log.theta_cmd(k) = lg.theta_cmd;
            update_ctr = 1;
        else
            % Hold previous values
            ctrl_log.s_alt(k)     = ctrl_log.s_alt(max(1,k-1));
            ctrl_log.s_lat(k)     = ctrl_log.s_lat(max(1,k-1));
            ctrl_log.s_att(k)     = ctrl_log.s_att(max(1,k-1));
            ctrl_log.theta_cmd(k) = ctrl_log.theta_cmd(max(1,k-1));
            update_ctr = update_ctr + 1;
        end

        F1_log(k) = F1;
        F2_log(k) = F2;

        % RK4 integration
        wind_k = wind_all(k,:)';
        state = RK4_step(quad, t(k), state, dt, F1, F2, wind_k);
        quad.state = state;
        states(k+1,:) = state';
    end

    % Fill last step
    F1_log(N) = F1;
    F2_log(N) = F2;
    ctrl_log.s_alt(N) = ctrl_log.s_alt(N-1);
    ctrl_log.s_lat(N) = ctrl_log.s_lat(N-1);
    ctrl_log.s_att(N) = ctrl_log.s_att(N-1);
    ctrl_log.theta_cmd(N) = ctrl_log.theta_cmd(N-1);
end

%%  Get metrics
function m = compute_metrics(states, F1, F2, x_cmd, z_cmd, t, dt)

    x = states(:,1);
    z = states(:,3);
    theta = states(:,5);
    N = length(t);

    ex = x_cmd - x;
    ez = z_cmd - z;

    m.rmse_x = sqrt(mean(ex.^2));
    m.rmse_z = sqrt(mean(ez.^2));
    m.max_err_x = max(abs(ex));
    m.max_err_z = max(abs(ez));
    m.mae_x = mean(abs(ex));
    m.mae_z = mean(abs(ez));
    m.max_pitch = max(abs(rad2deg(theta)));

    % Control effort
    m.effort = trapz(t, F1 + F2);

    % Thrust total variation
    m.thrust_TV = sum(abs(diff(F1))) + sum(abs(diff(F2)));

    % Settling time (5% of step = 0.05m)
    tol = 0.05;
    m.settle_x = find_settling(t, ex, tol);
    m.settle_z = find_settling(t, ez, tol);
end

%% Setling time helper
function ts = find_settling(t, error, tol)
    % Find last time the error leaves the tolerance band
    outside = abs(error) > tol;
    last_outside = find(outside, 1, 'last');
    if isempty(last_outside)
        ts = 0;
    elseif last_outside >= length(t)
        ts = Inf;  % Never settles
    else
        ts = t(last_outside);
    end
end
