classdef MovingAverage < handle
    
    properties
        sum = 0
        buffer = 0
        pointer = 0
    end
    
    methods
        function self = MovingAverage(N)
            self.buffer = zeros(N, 1);
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
                self.sum = self.sum + x_new / n - x_last / n;
                
                Y(i) = self.sum;
            end
        end
        
        function self = reset(self)
            self.buffer = self.buffer .* 0;
            self.sum = 0;
        end
    end
end
    