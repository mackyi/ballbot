

Qvector = [1/.5^2, 1/.5^2, 180^2/(10*pi)^2, 180^2/(10*pi)^2, 1/pi^2, 1/1.5^2,...
    1/1.5^2,180^2/(30*pi)^2, 180^2/(30*pi)^2, 1/(3*pi^2)];
Q= diag(Qvector);
Rv= [1/.9^2,1/.9^2,1/.9^2];
R = diag(Rv);
Alq = [0 0 0 -3.98434 0; 0 0 -3.98434 0 0; 0 0 17.7667 0 0;...
    0 0 0 17.7667 0; 0 0 0 0 0];

A = [zeros(5), eye(5); Alq, zeros(5)];

BBot = [0 -3.93462 3.93462; 4.54331 -2.27165 -2.27165;...
    -4.81295 2.40648 2.40648; 0 4.16814 -4.16814; 109.979 109.979 109.979];
B = [zeros(5,3); BBot];
K=lqr(A,B,Q,R);
%K=round(K*100000)/100000;
evector=[0 0 1 1 0 0 0 1 1 0];
BK = K*diag(evector);
testvector=[0 0 0 0 0 0 0 0 0 0];
M = [2/3*sqrt(2), 0, -sqrt(2)/3; -sqrt(2)/3, sqrt(6)/3,  -sqrt(2)/3; -sqrt(2)/3, -sqrt(6)/3, -sqrt(2)/3];
Minv = inv(M);
for psix=1:1
    testvector(3)=2*psix;
    testvector(4) = psix;
    testvector(9)= 0;
    testvector(8)= 0;
    BK*testvector'
end