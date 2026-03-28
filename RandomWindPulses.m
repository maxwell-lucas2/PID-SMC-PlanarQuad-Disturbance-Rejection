function wind = RandomWindPulses(t_vector, num_pulses, amp_range, dur_range)
    % Generate random wind disturbance (m/s) with specified pulses

    t = t_vector(:);
    N = length(t);
    wind = zeros(N, 1);
    rng(12345);

    for k = 1:num_pulses
        t0  = t(1) + (t(end) - t(1) - dur_range(2)) * rand;
        dur = dur_range(1) + (dur_range(2) - dur_range(1)) * rand;
        amp = amp_range(1) + (amp_range(2) - amp_range(1)) * rand;
        if rand < 0.5, amp = -amp; end

        start_idx = find(t >= t0, 1);
        end_idx   = find(t >= t0 + dur, 1);
        if isempty(end_idx), end_idx = N; end

        wind(start_idx:end_idx) = amp;
    end

    wind = wind(:);
end


%[appendix]{"version":"1.0"}
%---
