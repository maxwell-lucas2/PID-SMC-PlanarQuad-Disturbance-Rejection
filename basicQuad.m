%% Maxwell Lucas - University of Vermont Honors College Thesis
%% Basic model of a planar quadcopter

classdef basicQuad
    properties 
        m_motor  % mass of each motor [kg]
        m_body   % mass of body [kg]
        M        % total mass [kg]
        L        % arm length [m]
        J        % moment of inertia about pitch axis [kg*m^2]
        g = 9.81 % gravitational acceleration [m/s^2]
        state = zeros(6,1); % [x; dx; z; dz; theta; dtheta]
    end

    methods
        function obj = basicQuad()
            % Realistic small quadcopter parameters 
            obj.m_motor = 0.05;    % each motor
            obj.m_body  = 0.6;     % frame, electronics, battery
            obj.L       = 0.25;    % arm half-length (m)
            obj.M       = obj.m_body + 4*obj.m_motor; % total mass
            obj.J       = 2 * obj.m_motor * (obj.L^2) + 0.5 * obj.m_body * (0.1^2); 
            % approximate inertia about pitch axis
        end

        function d_State = update_state(obj, t, y, F1, F2, wind_input)
            % Computes state derivatives given current state, thrusts, and disturbances
            F1 = max(0, F1);
            F2 = max(0, F2);

            x = y(1); dx = y(2);
            z = y(3); dz = y(4);
            theta = y(5); dtheta = y(6);

            % Parse wind input 
            if length(wind_input) == 1
                wind_x = wind_input; wind_z = 0; tau_ext = 0;
            else
                wind_x = wind_input(1);
                wind_z = wind_input(2);
                tau_ext = wind_input(3);
            end

            % Total thrust and torque
            F_T = F1 + F2;
            tau = (F1 - F2) * obj.L;

            % Simple aerodynamic drag model
            rho = 1.225;
            Cd = 1.0;
            A = 0.015; % frontal area [m^2]

            F_wind_x = 0.5 * rho * Cd * A * wind_x * abs(wind_x);
            F_wind_z = 0.5 * rho * Cd * A * wind_z * abs(wind_z);

            % 2D quadrotor dynamics
            d_State = zeros(6,1);
            d_State(1) = dx;
            d_State(2) = (-F_T * sin(theta) + F_wind_x) / obj.M;
            d_State(3) = dz;
            d_State(4) = ( F_T * cos(theta) + F_wind_z) / obj.M - obj.g;
            d_State(5) = dtheta;
            d_State(6) = (tau + tau_ext) / obj.J;
        end
    end
end
