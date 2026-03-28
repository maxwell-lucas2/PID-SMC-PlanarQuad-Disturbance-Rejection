%% PID vs SMC — Wind Frequency Sweep Comparison
%  Maxwell Lucas — University of Vermont, Honors College Thesis
%  Applies a chirp wind signal (0.1–5 Hz) and compares tracking error
clear; close all; clc;

%% Sim parameters
dt       = 0.002;
ctrl_div = 5;
T_total  = 20.0;
t        = 0:dt:T_total;
N        = length(t);

x_cmd = 1.0;
z_cmd = 1.0;

% Generate frequency sweep wind
[~, wind_sweep] = WindFrequencySweep(t);
wind_all = [wind_sweep, zeros(N,1), zeros(N,1)];  % [Wx, Wz, tau]

clr_pid = [0.00 0.45 0.74];
clr_smc = [0.85 0.33 0.10];

%% ======================== RUN BOTH CONTROLLERS ==========================
[pid_states, pid_F1, pid_F2, ~] = run_sim('PID', x_cmd, z_cmd, t, dt, ctrl_div, wind_all);
[smc_states, smc_F1, smc_F2, smc_log] = run_sim('SMC', x_cmd, z_cmd, t, dt, ctrl_div, wind_all);

px = pid_states(:,1);  pz = pid_states(:,3);  pth = pid_states(:,5);
sx = smc_states(:,1);  sz = smc_states(:,3);  sth = smc_states(:,5);

%% ======================== FIGURE 1: TRACKING ============================
figure('Position', [50 50 1300 800], 'Name', 'Frequency Sweep — Tracking');

subplot(3,2,1);
plot(t, px, '-', 'Color', clr_pid, 'LineWidth', 1.3); hold on;
plot(t, sx, '-', 'Color', clr_smc, 'LineWidth', 1.3);
yline(x_cmd,'k--'); grid on; xlabel('Time [s]'); ylabel('X [m]');
title('Horizontal Position'); legend('PID','SMC','Cmd','Location','best');

subplot(3,2,2);
plot(t, pz, '-', 'Color', clr_pid, 'LineWidth', 1.3); hold on;
plot(t, sz, '-', 'Color', clr_smc, 'LineWidth', 1.3);
yline(z_cmd,'k--'); grid on; xlabel('Time [s]'); ylabel('Z [m]');
title('Vertical Position'); legend('PID','SMC','Cmd','Location','best');

subplot(3,2,3);
plot(t, abs(x_cmd - px), '-', 'Color', clr_pid, 'LineWidth', 1.0); hold on;
plot(t, abs(x_cmd - sx), '-', 'Color', clr_smc, 'LineWidth', 1.0);
grid on; xlabel('Time [s]'); ylabel('|Error| [m]');
title('X Error Magnitude'); legend('PID','SMC','Location','best');

subplot(3,2,4);
plot(t, abs(z_cmd - pz), '-', 'Color', clr_pid, 'LineWidth', 1.0); hold on;
plot(t, abs(z_cmd - sz), '-', 'Color', clr_smc, 'LineWidth', 1.0);
grid on; xlabel('Time [s]'); ylabel('|Error| [m]');
title('Z Error Magnitude'); legend('PID','SMC','Location','best');

subplot(3,2,5);
plot(t, rad2deg(pth), '-', 'Color', clr_pid, 'LineWidth', 1.2); hold on;
plot(t, rad2deg(sth), '-', 'Color', clr_smc, 'LineWidth', 1.2);
yline(0,'k--'); grid on; xlabel('Time [s]'); ylabel('Pitch [deg]');
title('Pitch Angle'); legend('PID','SMC','Location','best');

subplot(3,2,6);
plot(t, wind_sweep, 'r-', 'LineWidth', 1.0);
grid on; xlabel('Time [s]'); ylabel('Wind [m/s]');
title('Horizontal Wind Chirp (0.1–5 Hz)');

sgtitle('PID vs SMC — Frequency Sweep Disturbance', ...
    'FontSize', 14, 'FontWeight', 'bold');

%% ======================== FIGURE 2: WINDOWED RMSE =======================
% Compute RMSE in sliding windows to show performance vs frequency
win_sec  = 2.0;                          % Window length [s]
win_samp = round(win_sec / dt);
n_wins   = floor(N / win_samp);

win_t     = zeros(n_wins,1);
win_pid_x = zeros(n_wins,1);
win_smc_x = zeros(n_wins,1);
win_freq  = zeros(n_wins,1);

f0 = 0.1;  f1 = 5.0;  % Sweep range
for w = 1:n_wins
    idx = (w-1)*win_samp + 1 : w*win_samp;
    win_t(w) = mean(t(idx));
    win_freq(w) = f0 + (f1 - f0) * (win_t(w) / T_total);
    win_pid_x(w) = sqrt(mean((x_cmd - px(idx)).^2));
    win_smc_x(w) = sqrt(mean((x_cmd - sx(idx)).^2));
end

figure('Position', [100 100 900 400], 'Name', 'Windowed RMSE vs Frequency');

subplot(1,2,1);
plot(win_freq, win_pid_x, 'o-', 'Color', clr_pid, 'LineWidth', 1.5, 'MarkerFaceColor', clr_pid); hold on;
plot(win_freq, win_smc_x, 's-', 'Color', clr_smc, 'LineWidth', 1.5, 'MarkerFaceColor', clr_smc);
grid on; xlabel('Approx. Wind Frequency [Hz]'); ylabel('Windowed X RMSE [m]');
title('X Tracking RMSE vs Wind Frequency');
legend('PID','SMC','Location','best');

subplot(1,2,2);
plot(win_freq, win_pid_x ./ max(win_smc_x, 1e-6), 'k-o', 'LineWidth', 1.5, 'MarkerFaceColor', [0.5 0.5 0.5]);
yline(1, 'r--', 'LineWidth', 1); grid on;
xlabel('Approx. Wind Frequency [Hz]'); ylabel('RMSE Ratio (PID / SMC)');
title('Performance Ratio (>1 means SMC is better)');

sgtitle('Frequency-Dependent Disturbance Rejection', ...
    'FontSize', 13, 'FontWeight', 'bold');

%% ======================== PRINT SUMMARY =================================
fprintf('\n===== Frequency Sweep Summary =====\n');
fprintf('PID  X RMSE: %.4f m   Z RMSE: %.4f m\n', ...
    sqrt(mean((x_cmd-px).^2)), sqrt(mean((z_cmd-pz).^2)));
fprintf('SMC  X RMSE: %.4f m   Z RMSE: %.4f m\n', ...
    sqrt(mean((x_cmd-sx).^2)), sqrt(mean((z_cmd-sz).^2)));

%% ======================== SIMULATION FUNCTION ===========================
function [states, F1_log, F2_log, ctrl_log] = run_sim( ...
        ctrl_type, x_cmd, z_cmd, t, dt, ctrl_div, wind_all)

    N = length(t);
    quad = basicQuad();
    quad.state = [x_cmd; 0; z_cmd; 0; 0; 0];  % Start at target

    switch ctrl_type
        case 'PID',  ctrl = PIDCtrl(quad);
        case 'SMC',  ctrl = SMCCtrl(quad);
    end

    states = zeros(N, 6);
    F1_log = zeros(N, 1);
    F2_log = zeros(N, 1);
    ctrl_log.s_alt = zeros(N,1);
    ctrl_log.s_lat = zeros(N,1);
    ctrl_log.s_att = zeros(N,1);
    ctrl_log.theta_cmd = zeros(N,1);

    state = quad.state;
    states(1,:) = state';
    F1 = 0; F2 = 0;
    update_ctr = ctrl_div;

    for k = 1:N-1
        if update_ctr >= ctrl_div
            [F1, F2, lg] = ctrl.update(x_cmd, z_cmd, state);
            ctrl_log.s_alt(k)     = lg.s_alt;
            ctrl_log.s_lat(k)     = lg.s_lat;
            ctrl_log.s_att(k)     = lg.s_att;
            ctrl_log.theta_cmd(k) = lg.theta_cmd;
            update_ctr = 1;
        else
            ctrl_log.s_alt(k) = ctrl_log.s_alt(max(1,k-1));
            ctrl_log.s_lat(k) = ctrl_log.s_lat(max(1,k-1));
            ctrl_log.s_att(k) = ctrl_log.s_att(max(1,k-1));
            ctrl_log.theta_cmd(k) = ctrl_log.theta_cmd(max(1,k-1));
            update_ctr = update_ctr + 1;
        end

        F1_log(k) = F1;
        F2_log(k) = F2;

        wind_k = wind_all(k,:)';
        state = RK4_step(quad, t(k), state, dt, F1, F2, wind_k);
        quad.state = state;
        states(k+1,:) = state';
    end

    F1_log(N) = F1;  F2_log(N) = F2;
    ctrl_log.s_alt(N) = ctrl_log.s_alt(N-1);
    ctrl_log.s_lat(N) = ctrl_log.s_lat(N-1);
    ctrl_log.s_att(N) = ctrl_log.s_att(N-1);
    ctrl_log.theta_cmd(N) = ctrl_log.theta_cmd(N-1);
end
