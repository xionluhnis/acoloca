% Signal treatment

ori = dlmread('data/chirps_new3.log');
ori = dlmread('data/chirp_1.5k.log');
src = (ori-350)/250;

fprintf('\n');

f_bi = Biquad(Biquad.HIGHPASS, 1000, 1e4, 1);
filtshow('Biquadratic', f_bi.B, f_bi.A);
src_bi = f_bi.filter(src);

[B, A] = butter(3, 0.2, 'high');
filtshow('Butterworth 3rd', B, A);
src_bu = filter(B, A, src);

[B, A] = cheby1(3, 1, 0.2, 'high');
filtshow('Chebyshev I, 3rd', B, A);
src_ch1 = filter(B, A, src);

[B, A] = cheby2(3, 10, 0.2, 'high');
filtshow('Chebyshev II, 3rd', B, A);
src_ch2 = filter(B, A, src);

f_bi = Biquad(Biquad.BANDPASS, 1500, 1e4, 1);
filtshow('Biquad bandpass', f_bi.B, f_bi.A);
src_bip = f_bi.filter(src);

[B, A] = butter(3, [0.2, 0.4], 'bandpass');
filtshow('Butterworth bandpass', B, A);
src_bup = filter(B, A, src);

plots = {
    src, 'Signal';
    src_bi, 'Biquad';
    src_bu, 'Butterworth 3rd';
    src_ch1, 'Chebyshev I';
    src_ch2, 'Chebyshev II';
    src_bip, 'Biquad bandpass';
    src_bup, 'Butter bandpass'
};
N = size(plots, 1);
a = floor(sqrt(N));
b = ceil(N / a);

for i = 1:N
    subplot(a, b, i);
    plot(plots{i, 1});
    ylim([-1 1]);
    title(plots{i, 2});
end

fprintf('\n');

function filtshow(name, B, A)
    fprintf('%s:\n', name);
    fprintf('B: '); fprintf('%5.3f, ', B); fprintf('\n');
    fprintf('A: '); fprintf('%5.3f, ', A); fprintf('\n');
    fprintf('\n');
end