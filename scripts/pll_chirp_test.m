% Software PLL
clear all;
close all;

ideal_signal = 0;
sampling = 6.5e4:8e4;
%sampling = 4e4:7e4; % empty for all
fast_quad = 0;
sample_times = [];

% ori = dlmread('data/chirps_new3.log');
ori = dlmread('data/chirp_lfm2_1k_1.1k_s3000.log');

% sample timing
if size(ori, 2) == 2
    sample_times = ori(:, 1);
    ori = ori(:, 2);
end

% sample_rate = 0.86575e4; % from average timed serial reading
if isempty(sample_times)
    sample_rate = 0.86575e4; % quite important!
else
    s = floor(sample_times / 1e3);
    t_uniq = unique(s);
    t_uniq = t_uniq(2:end-1); % skip beggining and end timings (since partial)
    t_cnts = nan(numel(t_uniq), 1);
    for i = 1:numel(t_uniq)
        t_cnts(i) = numel(find(s == t_uniq(i)));
    end
    sample_rate = median(t_cnts);
end

pll_f0 = 1000;
pll_f1 = 1100;
pll_cf = (pll_f0 + pll_f1) / 2;

% chirp modulation
chirp_delta = pll_f1 - pll_f0;
chirp_duration = 1; % seconds
chirp_samples = ceil(chirp_duration * sample_rate);
sample_freq_shift = chirp_delta / (chirp_duration * sample_rate);

% sub-window processing
if ~isempty(sampling)
    ori = ori(sampling);
end

% normalize signal
if ideal_signal
    src = (ori-350)/200;
else
    % automatic gain control / normalization
    norm_size = ceil(3 * sample_rate / pll_cf);
    src = normalize(ori, norm_size);
    ori = ori(end-numel(src)+1:end);
end

% filter out-of-frequency noise
if ideal_signal
    % src = Biquad(Biquad.HIGHPASS, 500, 1e4, 1).filter(src);
    [B, A] = butter(3, [-0.01 +0.01] + pll_cf * 2 / sample_rate, 'bandpass');
    src = filter(B, A, src) * 2.5;
    ori = ori(end-numel(src)+1:end);
end

%% Sample from Arachnoid
% @see https://arachnoid.com/phase_locked_loop/index.html
% pll_loop_control = test_sig * ref_sig * pll_loop_gain # phase detector
% pll_loop_control = loop_lowpass(pll_loop_control) # loop low-pass filter
% output = output_lowpass(pll_loop_control) # output low-pass filter
% pll_integral += pll_loop_control / sample_rate # FM integral
% ref_sig = cos(2 * pi * pll_cf * (t + pll_integral)) # reference signal
% quad_ref = (ref_sig-old_ref) * sample_rate / (2 * pi * pll_cf) # quadrature reference
% old_ref = ref_sig
% pll_lock = lock_lowpass(-quad_ref * test_sig) # lock sensor
% logic_lock = (0,1)[pll_lock > 0.1] # logical lock

%% Matlab implementation



pll_integral = 0;
old_ref = 0;
old_ref1 = 0;
old_ref2 = 0;
old_pll_lock = 0;
pll_loop_gain = 0.1;
ref_sig = 0;
ref_idx = 1;

invsqr2 = 1.0 / sqrt(2.0);

loop_lowpass = Biquad(Biquad.IDENTITY, 1000, sample_rate, invsqr2);
output_lowpass = Biquad(Biquad.LOWPASS, 100, sample_rate, invsqr2);
lock_lowpass1 = Biquad(Biquad.LOWPASS, 10, sample_rate, invsqr2);
lock_lowpass2 = Biquad(Biquad.LOWPASS, 10, sample_rate, invsqr2);

% output for visualization
N = numel(src);
data = nan(N, 1);
locked_count = 0;
ref_freq = pll_cf;

for i = 1:N
    test_sig = src(i);
    t = i / sample_rate;
    
    % phase detector and filters
    pll_loop_control = test_sig * ref_sig * pll_loop_gain;
    pll_loop_control = loop_lowpass.filter(pll_loop_control);
    output = output_lowpass.filter(pll_loop_control);
    
    % FM integral
    pll_integral = pll_integral + pll_loop_control / sample_rate;
    
    % reference frequency
    if locked_count > 0
        ref_freq = pll_cf; % ref_freq + sample_freq_shift;
    end
    
    % reference signal
    ref_ph = 2 * pi * ref_freq * (t + pll_integral);
    ref_sig = sin(ref_ph);
    
    % quadrature
    if fast_quad
        quad_ref = (ref_sig - old_ref) * sample_rate / (2 * pi * ref_freq);
        old_ref = ref_sig;
    else
        quad_ref = cos(ref_ph);
    end
    
    % lock
    pll_lock1 = lock_lowpass1.filter(-quad_ref * test_sig);
    pll_lock2 = lock_lowpass2.filter(-ref_sig * test_sig);
    % logic lock uses four different phases
    logic_lock = max(abs(pll_lock1), abs(pll_lock2)) > 0.2; % thresholding
    if logic_lock
        if ~locked_count
            ref_freq = pll_f0;
        end
        locked_count = locked_count + 1;
    else
        locked_count = 0;
        ref_freq = pll_cf;
    end
    % logic_lock = max(abs(pll_lock1), abs(pll_lock2)) > 0.2;
    
    data(i, 1) = output - 0.5;
    data(i, 2) = pll_lock1;
    data(i, 3) = pll_lock2;
    data(i, 4) = logic_lock;
    data(i, 5) = pll_integral * 32; %pll_lock1 ^ 2 + pll_lock2 ^ 2;
    %data(i, 5) = (ref_freq - pll_cf) / pll_cf;
    % data(i, 2) = data(i,2) / (data(i,5)+1e-3);
end

x = reshape(1:N, [], 1);
p = plotyy(x, ori, repmat(x, 1, size(data, 2)), data);

p(1).Children(1).Color = 0.8 * [1 1 1];
% p(1).YLim = [-1 1];
p(2).YLim = [-1 1]; % * min(abs(p(2).YLim));
p(2).YTickMode = 'auto';
p(2).YGrid = 'on';

cols = parula(size(data, 2));
for i = 1:numel(p(2).Children)
    j = numel(p(2).Children)-i+1;
    l = p(2).Children(i);
    l.LineWidth = 1;
    l.Color = cols(end-i+1, :);
    if j == 2 || j == 3
        l.LineWidth = 2;
        l.LineStyle = ':';
    else
        l.LineStyle = '-';
    end
    if j == 4
        l.LineWidth = 3;
    end
end
set(gcf, 'color', 'w');
l = legend('Signal', ...
    'PLL Output', 'Lock1', 'Lock2', 'Logic Lock', ...
    'PLL Integral', 'Relative Frequency');
l.FontSize = 16;
