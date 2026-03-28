% SMC.m – Sliding Mode Controller with boundary layer
classdef SMC
    properties
        lambda
        k
        phi      % Boundary layer thickness
        Tc
        MorJ     % Mass or moment of inertia
    end

    methods
        function obj = SMC(lambda, k, phi, Tc, MorJ)
            obj.lambda = lambda;
            obj.k = k;
            obj.phi = phi;
            obj.Tc = Tc;
            obj.MorJ = MorJ;
        end

        function [u_eq, u_sw, s] = update(obj, e, e_dot, dd_des)
            s = e_dot + obj.lambda * e;
            u_eq = obj.MorJ * (dd_des + obj.lambda * e_dot);
            u_sw = obj.MorJ * obj.k * sat(s / obj.phi);
        end
    end
end

% Smooth saturation function
function y = sat(x)
    y = zeros(size(x));
    idx = abs(x) <= 1;
    y(idx) = x(idx);
    y(~idx) = sign(x(~idx));
end
