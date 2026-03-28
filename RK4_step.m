function [y_next] = RK4_step(quad, t, y, h, F1, F2, wind_speed_x)

    k1 = quad.update_state(t, y, F1, F2, wind_speed_x);
    k2 = quad.update_state(t + 0.5*h, y + 0.5*h*k1, F1, F2, wind_speed_x);
    k3 = quad.update_state(t + 0.5*h, y + 0.5*h*k2, F1, F2, wind_speed_x);
    k4 = quad.update_state(t + h, y + h*k3, F1, F2, wind_speed_x);

    y_next = y + (h/6)*(k1 + 2*k2 + 2*k3 + k4);

    % Ensure column vector
    y_next = y_next(:);
end
