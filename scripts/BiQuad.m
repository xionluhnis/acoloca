classdef BiQuad < handle
   
    properties (Constant)
        LOWPASS = 0
        HIGHPASS = 1
        BANDPASS = 2
        NOTCH = 3
        PEAK = 4
        LOWSHELF = 5
        HIGHSHELF = 6
    end
    
    properties
        srate = 0
        A = [0, 0, 0]
        B = [0, 0, 0]
        x = [0, 0]
        y = [0, 0]
    end
    
    methods
        function self = BiQuad(type, freq, srate, Q, dbGain)
            
            freq = single(freq);
            self.srate = single(srate);
            Q = single(Q);
            dbGain = single(dbGain);
            
            % parameters
            A = 10 ^ (dbGain / 40);
            omega = 2 * pi * freq / self.srate;
            sn = sin(omega);
            cs = cos(omega);
            alpha = sn / (2*Q);
            beta = sqrt(A + A);
            
            switch type
                case BiQuad.LOWPASS
                    self.lowpass(cs, alpha);
                case BiQuad.HIGHPASS
                    self.highpass(cs, alpha);
                case BiQuad.BANDPASS
                    self.bandpass(cs, alpha);
                case BiQuad.NOTCH
                    self.notch(cs, alpha);
                case BiQuad.PEAK
                    self.peak(A, cs, alpha);
                case BiQuad.LOWSHELF
                    self.lowshelf(A, sn, cs, beta);
                case BiQuad.HIGHSHELF
                    self.highshelf(A, sn, cs, beta);
                otherwise
                    error('Invalid type %d', type)
            end
            
            % prescaling
            self.B = self.B / self.A(1);
            self.A = self.A / self.A(1);
        end
        
        function self = lowpass(self, cs, alpha)
            self.B = [(1 - cs) / 2, 1-cs, (1-cs)/2];
            self.A = [1+alpha, -2*cs, 1-alpha];
        end

        function self = highpass(self, cs, alpha)
            self.B = [(1+cs)/2, -(1+cs), (1+cs)/2];
            self.A = [1+alpha, -2*cs, 1-alpha];
        end

        function self = bandpass(self, cs, alpha)
            self.B = [alpha, 0, -alpha];
            self.A = [1+alpha, -2*cs, 1-alpha];
        end

        function self = notch(self, cs, alpha)
            self.B = [1, -2*cs, 1];
            self.A = [1+alpha, -2*cs, 1-alpha];
        end

        function self = peak(self, A, cs, alpha)
            self.B = [1+alpha*A, -2*cs, 1-alpha*A];
            self.A = [1+alpha/A, -2*Cs, a-alpha/A];
        end

        function self = lowshelf(self, A, sn, cs, beta)
            self.B = [ ...
                A * ((A + 1) - (A - 1) * cs + beta * sn), ...
                2 * A * ((A - 1) - (A + 1) * cs), ...
            	A * ((A + 1) - (A - 1) * cs - beta * sn) ...
            ];
            self.A = [ ...
            	(A + 1) + (A - 1) * cs + beta * sn, ...
            	-2 * ((A - 1) + (A + 1) * cs), ...
            	(A + 1) + (A - 1) * cs - beta * sn ...
            ];
        end

        function self = highshelf(self, A, sn, cs, beta)
            self.B = [ ...
            	A * ((A + 1) + (A - 1) * cs + beta * sn), ...
            	-2 * A * ((A - 1) + (A + 1) * cs), ...
            	A * ((A + 1) + (A - 1) * cs - beta * sn) ...
            ];
            self.A = [ ...
                (A + 1) - (A - 1) * cs + beta * sn, ...
            	2 * ((A - 1) - (A + 1) * cs), ...
            	(A + 1) - (A - 1) * cs - beta * sn ...
            ];
        end
        
        function y = subsindex(self, x)
            %y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2 - self.a1 * self.y1 - self.a2 * self.y2
            y = self.B .* [x, self.x] - self.A(2:3) .* self.y;
            % shift memory
            self.x = [x, self.x(1)];
            self.y = [y, self.y(1)];
        end
        
        function v = result(self, f)
            phi = sin(pi * f * 2/(2.0 * self.srate)) .^ 2;
            v = ( ...
                  sum(self.B)^2 ...
                - 4*(self.B(1) * self.B(2) + 4*self.B(1)*self.B(3) ...
                     + self.B(2)*self.B(3)) * phi ...
                + 16*self.B(1)*self.B(3) * phi .* phi ...
                ) ./ ( ...
                  sum(self.A)^2 ...
                - 4*(self.A(2) + 4*self.A(3) + self.A(2)*self.A(3)) * phi ...
                + 16*self.A(3)*phi .*phi ...
            );
        end
        
        function r = log_result(self, f)
            try
                r = 10 * log10(self.result(f));
            catch e
                r = -200;
            end
            if isnan(r)
                r = -200;
            end
        end
        
        function C = constants(self)
            C = [self.B, self.A(2:3)];
        end
    end
end
    