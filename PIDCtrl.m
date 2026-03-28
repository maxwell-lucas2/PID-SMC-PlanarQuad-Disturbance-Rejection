% Maxwell Lucas — University of Vermont, Honors College Thesis
% PID Controller for single planar quadrotor disturbance rejection
% Standardized interface: [F1, F2, log] = update(obj, x_cmd, z_cmd, state)

classdef PIDCtrl < handle
    properties
        altCtrl     % Altitude PID
        latCtrl     % Lateral PID
        attCtrl     % Attitude PID

        quad        % Reference to basicQuad for physical params

        theta_max   % Pitch command limit [rad]
        F_min       % Motor thrust floor [N]
        F_max       % Motor thrust ceiling [N]
    end

    methods
        function obj = PIDCtrl(quad)
            obj.quad = quad;
            obj.theta_max = deg2rad(45);  % Matched to SMC
            obj.F_min = 0;
            obj.F_max = 20;

            %  Gains: [Kp, Ki, Kd, Kt_antiwindup, Tau_filter, Tc]
            Tc = 0.01;  % Controller update period
            obj.altCtrl = PID(6.0, 2.0, 5.0, 0.05, 0.05, Tc);
            obj.latCtrl = PID(4.5, 0.5, 2.0, 0.50, 0.05, Tc);
            obj.attCtrl = PID(30.0, 0.0, 5.0, 0.50, 0.02, Tc);
        end

        function [F1, F2, log] = update(obj, x_cmd, z_cmd, state)
            x = state(1);
            z = state(3);
            theta = state(5);

            % Altitude
            u_z = obj.altCtrl.update(z_cmd, z);
            FT  = obj.quad.M * (obj.quad.g + u_z);

            % Lateral -> desired pitch
            a_x_des   = obj.latCtrl.update(x_cmd, x);
            theta_cmd = -atan2(a_x_des, obj.quad.g + u_z);
            theta_cmd = max(-obj.theta_max, min(obj.theta_max, theta_cmd));

            % Attitude
            alpha_des = obj.attCtrl.update(theta_cmd, theta);
            torque    = obj.quad.J * alpha_des;

            % Motor mixing
            F1 = 0.5 * (FT + torque / obj.quad.L);
            F2 = 0.5 * (FT - torque / obj.quad.L);
            F1 = max(obj.F_min, min(obj.F_max, F1));
            F2 = max(obj.F_min, min(obj.F_max, F2));

            % Log
            log.F1 = F1;
            log.F2 = F2;
            log.FT = F1 + F2;
            log.theta_cmd = theta_cmd;
            log.s_alt = 0; 
            log.s_lat = 0;
            log.s_att = 0;
        end
    end
end
