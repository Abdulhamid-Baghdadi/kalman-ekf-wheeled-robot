A = [1 0 0; 0 1 0; 0 0 1]
R = [0.1 0 0; 0 0.1 0; 0 0 0.1]
Q = [0.1 0 0; 0 0.1 0; 0 0 0.1]
C = eye(3)
Sigma = zeros(3)

i = 10
while(i > 1)
   Sigma = A*Sigma*A' + R;
   K = Sigma * C' * (C * Sigma * C' + Q)^-1;
   Sigma = (eye(3) - K*C)*Sigma;
   i = i - 1;
end

Sigma
