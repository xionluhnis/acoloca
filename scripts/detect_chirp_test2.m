% Find chirp timing

if ~exist('data', 'var')
    pll_chirp_test2
end

locked = data(:, 4);
output = data(:, 1);
if mean(output) < -0.2
    output = output + 0.5;
end
delta = data(:, 5);

% wma
WMA_N = round(chirp_samples / 8);
output_wma = WeightedMovingAverage(WMA_N);
diff_wma   = WeightedMovingAverage(WMA_N);

% isolate largest continuous lock
lock_events = find([0; diff(locked)]);
lock_starts = lock_events(1:2:end);
lock_ends   = lock_events(2:2:end);
lock_durations = lock_ends - lock_starts + 1;
[~, main_lock] = max(lock_durations);
start_lock = lock_starts(main_lock);
end_lock   = lock_ends(main_lock)-1;
range = start_lock:end_lock;
start_read = start_lock + WMA_N;

delta_wma  = zeros(numel(delta), 1);
delta_wma(range) = diff_wma.filter(delta(range));

zcross = [0; diff(delta_wma > 0)];
zcross(1:start_read-1) = 0;

X = reshape(1:numel(delta), [], 1);

yyaxis left;
hold off;
plot(X, ori, ':', 'color', [0.6, 0.9, 1], 'DisplayName', 'Signal');

yyaxis right;
hold off;
plot(X, output / 1000, ':', 'color', [0.6, 1, 0.3] * 0.7, 'DisplayName', 'PLL Output');
hold on;
plot(X, delta_wma, 'color', [1 1 1] * 0.6, 'DisplayName', 'PLL Output Delta');

hold on;
mark = @(x, c, m, text)mark_seq(x, delta_wma, c, m, text);

mark(start_lock, [0.3, 1, 0.6] * 0.4, 'o', 'Lock Start');
mark(end_lock,   [0.3, 1, 0.6] * 0.4, 'p', 'End Start');
mark(start_read, [0.3, 1, 0.6] * 0.8, 'o', 'Read Start');
mark(find(zcross),  [1, 0.6, 0.3],    'd', 'Zero Crossing'); %#ok<FNDSB>
% mark(find(zcross) + WMA_N*0.75, 'r',  's', 'Safe Start');
mark(find(zcross) - WMA_N*0.75, 'b',  's', 'Safe End');

% estimate center from two sides
l_start = start_read;
r_start = ceil(find(zcross, 1));
l_end   = floor(r_start - WMA_N*0.75);
r_end   = min(end_lock , find(zcross .* (X > r_start + WMA_N/2), 1)) ...
            - ceil(WMA_N*0.75);

l_mean = (l_start+l_end)/2;
r_mean = (r_start+r_end)/2;
l_out_mean = mean(output(l_start:l_end));
r_out_mean = mean(output(r_start:r_end));
l_delta_mean = mean(delta(l_start:l_end));
r_delta_mean = mean(delta(r_start:r_end));

l_mdl = fitlm(l_start:l_end, output(l_start:l_end));
r_mdl = fitlm(r_start:r_end, output(r_start:r_end));

% y = y_mean + (t_mean-x_mean) * d_mean
%
% => l_out_mean + (t_est - l_mean) * l_delta_mean
%  = r_out_mean + (t_est - r_mean) * r_delta_mean
%
% => t_est = (l_out_mean - r_out_mean + l_mean * l_delta_mean - r_mean * r_delta_mean) / (r_delta_mean - l_delta_mean)
delta_out_mean = l_out_mean - r_out_mean;
delta_delta_mean = l_delta_mean - r_delta_mean;
t_est = (l_mean * l_delta_mean - r_mean * r_delta_mean ...
                    - delta_out_mean) / delta_delta_mean;
% estimate by inverting the slopes (should be opposite of the other ideally)
t_inv = (l_mean * r_delta_mean - r_mean * l_delta_mean ...
                    - delta_out_mean) / -delta_delta_mean;
                
% 1/4 and 3/4 zero crossings of output
out_zx = [0; diff(output > 0)];
out_zx(1:start_lock) = 0;
out_zx(end_lock:end) = 0;
mark(find(out_zx ~= 0), 'k', 'd', 'Output Zero Crossing');
t_ozx = mean(find(out_zx ~= 0));

y_range = [min(delta_wma) max(delta_wma)];
plot([1 1] * t_est, y_range, '--b', 'DisplayName', 'Direct Estimate');
plot([1 1] * t_inv, y_range, '--r', 'DisplayName', 'Reverse Estimate');
plot([1 1] * t_ozx, y_range, 'o-.k', 'DisplayName', 'OZX Estimate');

% Estimate variability
sample_range = abs(t_est - t_inv);
time_range   = sample_range / sample_rate;
dist_range   = time_range * 343.0;
fprintf('Sample range: %g\n', sample_range);
fprintf('Time   range: %g [s]\n', time_range);
fprintf('Dist   range: %g [mm]\n', dist_range * 1000);
fprintf('Delta left: %g\n', l_delta_mean);
fprintf('Delta right: %g\n', r_delta_mean);
fprintf('Times: %g\n', [t_est, t_inv, t_ozx]);
fprintf('Dist (est|ozx): %g [mm]\n', 343 * abs(t_ozx - t_est) / sample_rate * 1000);

legend('location', 'southwest');
set(gcf, 'color', 'w');

function p = mark_seq(X, Y, color, marker, text)
    if islogical(X)
        X = find(X);
    elseif isfloat(X)
        X = round(X);
    end
    if numel(Y) < X
        assert(numel(Y) == 1);
        Y = repmat(Y, size(X, 1), size(X, 2));
    elseif numel(Y) > X
        Y = Y(X);
    end
    scatter(X, Y, 'markeredgecolor', color, 'markerfacecolor', color, ...
        'markerfacealpha', 0.5, ...
        'marker', marker, 'sizedata', 100, ...
        'DisplayName', text);
end