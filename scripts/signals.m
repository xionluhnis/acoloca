% Signal treatment

ori = dlmread('data/chirps_new3.log');
src = (ori-350)/250;

fprintf('\n');

f_bi = Biquad(Biquad.HIGHPASS, 1000, 1e4, 1);
filtshow('Biquadratic', f_bi.B, f_bi.A);
src_bi = f_bi.filter(src);

[B, A] = butter(3, 0.1, 'high');
filtshow('Butterworth 3rd', B, A);
src_bu = filter(B, A, src);

[B, A] = cheby1(3, 1, 0.1, 'high');
filtshow('Chebyshev I, 3rd', B, A);
src_ch1 = filter(B, A, src);

[B, A] = cheby2(3, 10, 0.1, 'high');
filtshow('Chebyshev II, 3rd', B, A);
src_ch2 = filter(B, A, src);

subplot(151); plot(src); ylim([-1 1]);
subplot(152); plot(src_bi); ylim([-1 1]); % biquadratic
subplot(153); plot(src_bu); ylim([-1 1]); % butterworth 3rd order
subplot(154); plot(src_ch1); ylim([-1 1]); % chebyshev I 3rd order
subplot(155); plot(src_ch2); ylim([-1 1]); % chebyshev II 3rd order

fprintf('\n');

function filtshow(name, B, A)
    fprintf('%s:\n', name);
    fprintf('B: '); fprintf('%5.3f, ', B); fprintf('\n');
    fprintf('A: '); fprintf('%5.3f, ', A); fprintf('\n');
    fprintf('\n');
end