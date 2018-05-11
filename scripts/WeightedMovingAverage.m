classdef WeightedMovingAverage < handle
    
    properties
        total = 0
        numerator = 0
        denum_const = 1
        buffer = 0
        pointer = 0
    end
    
    methods
        function self = WeightedMovingAverage(N)
            self.buffer = zeros(N, 1);
            self.denum_const = N*(N+1)/2;
        end
        
        function Y = filter(self, X)
            assert(isnumeric(X), 'Filter arguments must be numeric');
            assert(numel(size(X)) == 2, 'No N-D argument');
            Y = nan(size(X, 1), size(X, 2));
            for i = 1:numel(X)
                x_new = X(i);
                
                % update ring buffer
                n = length(self.buffer);
                ptr_new = mod(self.pointer + 1, n);
                x_last = self.buffer(1 + ptr_new);
                self.pointer = ptr_new;
                self.buffer(1 + ptr_new) = x_new;
                
                % compute new output
                total_m = self.total;
                numerator_m = self.numerator;
                self.total = total_m + x_new - x_last;
                self.numerator = numerator_m + n * x_new - total_m;
                
                Y(i) = self.numerator / self.denum_const;
            end
        end
        
        function self = reset(self)
            self.buffer = self.buffer .* 0;
            self.total = 0;
            self.numerator = 0;
            self.pointer = 0;
        end
    end
end
    