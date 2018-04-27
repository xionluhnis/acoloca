function [y, u, A] = normalize(x, w, keep_first)
%NORMALIZE Normalizes the amplitude of a signal over time

    if ~exist('w', 'var')
        w = 10;
    end
    if ~exist('keep_first', 'var')
        keep_first = 0;
    end
    N = numel(x);
%     X = zeros(N, w);
%     for i = 1:w
%         delta = i-1;
%         X(:, i) = x(1:w);
%     end
    u = zeros(N, 1);
    A = ones(N, 1);
    A(1:w-1) = 1; % default amplitude
    for i = w:N
        W = x(i-w+1:i);
%         m = min(W);
%         M = max(W);
%         u(i) = (m + M) / 2;
%         A(i) = (M - m) / 2;
        u(i) = mean(W);
        A(i) = range(W) / 2;
        
    end

    y = (x(:) - u) ./ A;
    if ~keep_first
        y = y(w:end);
        u = u(w:end);
        A = A(w:end);
    end
end

