function metrics = analyze_step_response(t, y, y0, y1)
%ANALYZE_STEP_RESPONSE Basic rise-time and overshoot metrics.

amp = y1 - y0;
if abs(amp) < 1e-9
    metrics.rise_time = NaN;
    metrics.overshoot_pct = 0;
    return;
end

y10 = y0 + 0.1*amp;
y90 = y0 + 0.9*amp;

i10 = find((y - y10)*sign(amp) >= 0, 1, 'first');
i90 = find((y - y90)*sign(amp) >= 0, 1, 'first');

if isempty(i10) || isempty(i90)
    metrics.rise_time = NaN;
else
    metrics.rise_time = t(i90) - t(i10);
end

peak = max((y - y1)*sign(amp));
metrics.overshoot_pct = max(0, 100*peak/abs(amp));

end
