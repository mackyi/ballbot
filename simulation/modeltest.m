

Qvector = [1/.5^2, 1/.5^2, 180^2/(10*pi)^2, 180^2/(10*pi)^2, 1/pi^2, 1/1.5^2,...
    1/1.5^2,180^2/(30*pi)^2, 180^2/(30*pi)^2, 1/(3*pi^2)];
Q= diag(Qvector);
Rv= [1/.9^2,1/.9^2,1/.9^2];
R = diag(Rv);
Alq = [0 0 0 -46.3594 0; 0 0 -46.3594 0 0; 0 0 146.362 0 0;...
    0 0 0 146.362 0; 0 0 0 0 0];

A = [zeros(5), eye(5); Alq, zeros(5)];

BBot = [0 -16.5131 16.5131; 19.0676 -9.53381 -9.53381;...
    -50.7019 25.3509 25.3509; 0 43.9091 -43.9091; 61.3267 61.3267 61.3267];
B = [zeros(5,3); BBot];
K=lqr(A,B,Q,R);
%K=round(K*100000)/100000;
evector=[0 0 1 1 0 0 0 1 1 0];
BK = K*diag(evector);
testvector=[0 0 0 0 0 0 0 0 0 0];
M = [2/3*sqrt(2), 0, -sqrt(2)/3; -sqrt(2)/3, sqrt(6)/3,  -sqrt(2)/3; -sqrt(2)/3, -sqrt(6)/3, -sqrt(2)/3];
Minv = inv(M);
for psix=0:.01:.2
    testvector(3)=2*psix;
    testvector(4) = psix;
    testvector(9)= 0;
    testvector(8)= 0;
    angle = psix*180/pi 
    T=BK*testvector'.*14
end