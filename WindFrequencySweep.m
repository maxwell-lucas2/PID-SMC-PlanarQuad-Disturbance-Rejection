function [t, wind] = WindFrequencySweep(t_vector)
    % Maxwell Lucas, 10-13-2025
    % Updated for smooth frequency sweep without pulse-like gusts

    % Wind frequency sweep parameters
    A = 1;       % Base amplitude [m/s]
    f0 = 0.1;    % Start frequency [Hz]
    f1 = 5;      % End frequency [Hz]

    % Time vector
    t = t_vector(:);  % ensure column vector
    
    % Smooth frequency sweep
    f_t = f0 + (f1 - f0) * (t / t(end));  % Linearly ramp frequency
    
    % Integrate to get smooth phase evolution
    phase = 2 * pi * cumtrapz(t, f_t);

    % Smooth wind signal
    wind = A * sin(phase);
end
