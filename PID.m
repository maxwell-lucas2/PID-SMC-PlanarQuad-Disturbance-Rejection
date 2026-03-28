% Maxwell Lucas, 10-9-2025
% University of Vermont, Honors College Thesis

% PID Class with back calculated anti-windup and derivative filtering

classdef PID < handle

    properties
        % Gains
        Kp; Ki; Kd;

        % Anti-windup
        Kt; % Anti-windup gain
        u_min;
        u_max;
        integrator = 0;

        % Derivative filtering 
        Tau; % Time constant for filter
        prev_error = 0;
        prev_filt_deriv = 0;

        Tc;  % Controller time step
    end

    methods
        function obj = PID(Kp,Ki,Kd,Kt,Tau,Tc)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.u_min = -Inf;
            obj.u_max = Inf;
            obj.Tau = Tau;
            obj.Tc = Tc;
            obj.Kt = Kt;
        end

        function u = update(obj, desired, actual)
            % Get error 
            e = desired - actual;

            % Proportional term
            P = obj.Kp * e;

            % Filtered derivative term
            unfilt_deriv = (e - obj.prev_error) / obj.Tc;
            alpha = (obj.Tc / (obj.Tau + obj.Tc));
            filt_deriv = alpha * unfilt_deriv + (1 - alpha) * obj.prev_filt_deriv;
            D = obj.Kd * filt_deriv;

            % Integrator
            obj.integrator = obj.integrator + e * obj.Tc;
            I_raw = obj.Ki * obj.integrator;

            % unsaturated control
            u_unsat = P + I_raw + D;

            % saturate
            u = min(max(u_unsat, obj.u_min), obj.u_max);

            % back-calculation anti-windup 
            obj.integrator = obj.integrator - obj.Kt * (u_unsat - u);

            % store filter state
            obj.prev_filt_deriv = filt_deriv;
            obj.prev_error = e;
        end
    end
end



