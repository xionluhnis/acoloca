% Software PLL
clear all;

ori = dlmread('data/chirps_new3.log');
ori = dlmread('data/chirp_1.5k.log');
% subsample
src = (ori-350)/200;
% src = src(7e4:12e4);
ori = ori(end-numel(src)+1:end);

%src = normalize(ori); % idealized
%ori = ori(end-numel(src)+1:end);
% src = Biquad(Biquad.HIGHPASS, 500, 1e4, 1).filter(src);
%[B, A] = butter(3, [0.2, 0.4], 'bandpass');
%src = filter(B, A, src);
ori = ori(end-numel(src)+1:end);

%src = src .* (abs(src) > 0.1);

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

% sample_rate = 0.86575e4; % from average timed serial reading
sample_rate = 0.9e4; % from average timed serial reading

f_start = 1500;
f_end   = 1500;

pll_integral = 0;
old_ref = 0;
old_ref1 = 0;
old_ref2 = 0;
old_pll_lock = 0;
pll_cf = 1500;
pll_loop_gain = 1;
ref_sig = 0;
ref_idx = 1;

invsqr2 = 1.0 / sqrt(2.0);

loop_lowpass = Biquad(Biquad.LOWPASS, 1000, sample_rate, invsqr2);
output_lowpass = Biquad(Biquad.IDENTITY, 100, sample_rate, invsqr2);
lock_lowpass1 = Biquad(Biquad.LOWPASS, 10, sample_rate, invsqr2);
lock_lowpass2 = Biquad(Biquad.LOWPASS, 10, sample_rate, invsqr2);

% output for visualization
N = numel(src);
da = nan(N, 1);
db = nan(N, 1);
dc = nan(N, 1);
dd = nan(N, 1);
de = nan(N, 1);
df = nan(N, 1);

S1 = [];
S2 = [];

data = nan(N, 1);

for i = 1:N
    test_sig = src(i);
    t = i / sample_rate;
    
    % phase detector and filters
    pll_loop_control = test_sig * ref_sig * pll_loop_gain;
    pll_loop_control = loop_lowpass.filter(pll_loop_control);
    output = output_lowpass.filter(pll_loop_control);
    
    % FM integral
    pll_integral = pll_integral + pll_loop_control / sample_rate;
    % pll_integral = 0;
    
    % reference signal
    ref_ph = 2 * pi * pll_cf * (t + pll_integral);
    ref_sig = sin(ref_ph);
    S1(i) = ref_sig;
    
    % quadrature
    %quad_ref = (ref_sig - old_ref) * sample_rate / (2 * pi * pll_cf);
    %old_ref = ref_sig;
    quad_ref = cos(ref_ph);
    
    % lock
    pll_lock1 = lock_lowpass1.filter(-quad_ref * test_sig);
    pll_lock2 = lock_lowpass2.filter(-ref_sig * test_sig);
    % logic lock uses four different phases
    logic_lock = max(abs(pll_lock1), abs(pll_lock2)) > 0.2; % thresholding
    
    data(i, 1) = output - 0.5;
    data(i, 2) = pll_lock1;
    data(i, 3) = pll_lock2;
    data(i, 4) = logic_lock;
    data(i, 5) = pll_integral;
end

x = reshape(1:N, [], 1);
p = plotyy(x, src, repmat(x, 1, size(data, 2)), data);

p(1).Children(1).Color = 0.8 * [1 1 1];
p(1).YLim = [-1 1];
p(2).YLim = [-1 1]; % * min(abs(p(2).YLim));
p(2).YTickMode = 'auto';
p(2).YGrid = 'on';

cols = parula(size(data, 2));
for i = 1:numel(p(2).Children)
    l = p(2).Children(i);
    l.LineWidth = 3;
    l.Color = cols(end-i+1, :);
    if i <= 1
        l.LineWidth = 1;
    elseif i > 3
        l.LineStyle = ':';
    end
end
legend('Signal', ...
    'PLL Output', 'Lock1', 'Lock2', 'Logic Lock', ...
    'PLL Integral');