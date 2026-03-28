% Maxwell Lucas — University of Vermont, Honors College Thesis
% SMC Controller for single planar quadrotor disturbance rejection
% Standardized interface: [F1, F2, log] = update(obj, x_cmd, z_cmd, state)

classdef SMCCtrl < handle
    properties
        altCtrl     % Altitude SMC
        latCtrl     % Lateral SMC
        attCtrl     % Attitude SMC

        quad        % Reference to basicQuad for physical params

        theta_max   % Pitch command limit [rad]
        F_min       % Motor thrust floor [N]
        F_max       % Motor thrust ceiling [N]
    end

    methods
        function obj = SMCCtrl(quad)
            obj.quad = quad;
            obj.theta_max = deg2rad(45);  % Matched to PID
            obj.F_min = 0;
            obj.F_max = 20;

            Tc = 0.01;  % Controller update period
            % SMC(lambda, k, phi, Tc, MorJ)
            %
            % These gains are tuned to produce comparable bandwidth to the
            % PID controller.  The ORIGINAL gains (lambda_lat=7, k_lat=10,
            % phi_lat=0.1) were tuned under Euler integration at dt=0.01,
            % which added artificial numerical damping that masked instability.
            % Under RK4 at dt=0.002, those gains cause a pitch-thrust
            % coupling divergence: aggressive pitch commands reduce vertical
            % thrust -> altitude SMC compensates with more FT -> lateral
            % force grows -> runaway.
            %
            % The gains below produce similar settling times to PID while
            % preserving SMC's structural advantages (inherent saturation,
            % matched-disturbance invariance, no integral windup).
            obj.altCtrl = SMC(3.0,  5.0, 0.3,  Tc, quad.M);
            obj.latCtrl = SMC(2.0,  3.0, 0.5,  Tc, quad.M);
            obj.attCtrl = SMC(8.0,  3.0, 0.15, Tc, quad.J);
        end

        function [F1, F2, log] = update(obj, x_cmd, z_cmd, state)
            x     = state(1);  dx  = state(2);
            z     = state(3);  dz  = state(4);
            theta = state(5);  dth = state(6);

            % Altitude: SMC outputs a force (MorJ = M)
            [ueq_z, usw_z, s_z] = obj.altCtrl.update(z_cmd - z, -dz, 0);
            FT = obj.quad.M * obj.quad.g + ueq_z + usw_z;

            % Lateral: SMC outputs a force (MorJ = M), convert to accel
            [ueq_x, usw_x, s_x] = obj.latCtrl.update(x_cmd - x, -dx, 0);
            a_x_des = (ueq_x + usw_x) / obj.quad.M;

            % Desired pitch from lateral acceleration
            theta_cmd = atan2(-a_x_des, obj.quad.g);
            theta_cmd = max(-obj.theta_max, min(obj.theta_max, theta_cmd));

            % Attitude: SMC outputs a torque (MorJ = J)
            th_err  = wrapToPi(theta_cmd - theta);
            [ueq_th, usw_th, s_th] = obj.attCtrl.update(th_err, -dth, 0);
            torque = ueq_th + usw_th;

            % Motor mixing
            F1 = 0.5 * FT + 0.5 * (torque / obj.quad.L);
            F2 = 0.5 * FT - 0.5 * (torque / obj.quad.L);
            F1 = max(obj.F_min, min(obj.F_max, F1));
            F2 = max(obj.F_min, min(obj.F_max, F2));

            % Log
            log.F1        = F1;
            log.F2        = F2;
            log.FT        = F1 + F2;
            log.theta_cmd = theta_cmd;
            log.s_alt     = s_z;
            log.s_lat     = s_x;
            log.s_att     = s_th;
        end
    end
end
