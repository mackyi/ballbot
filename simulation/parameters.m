ms = 3.2;
rs = 0.115;
is = 2.65e-2;
2*ms*rs*rs/3
l = 0.405;
g = 9.81;
alpha = pi/4;
beta = 0;
mw = 0.995;
mb = 7.135;
rw = 0.05;
iw = 1.9e-3;
ib = 2.4;


mb1 = 6.2;
mb2 = 3.8;
h = 0.8;
h1 = 0.66;
h2 = 0.14;

l = (mb1*(rs+h-h1/2)+mb2*(rs+h2/2))/(mb1+mb2)

lambda = mw*(rs+rw)+mb*l;
mtot = ms + mw+ mb;
for rb=0
    rtot = rs+rb;

    dG = [0; -lambda*g];
    M00 = is + rs^2*mtot+rs^2/(rw^2)*iw;
    M01 = rs*lambda+rs^2/(rw^2)*iw;
    M10 = rs*lambda+rs^2/(rw^2)*iw;
    M11 = rtot^2*mw+rs^2/(rw^2)*iw + ib;
    C=-1*(.010051-.524434);
    M = [M00 C; C 2.43714]
    inv(M);
    A = inv(M)*-dG
     AA = [-75.5784; 34.1594];
     AA./A  
%     dG*A'*inv(A*A');
%     
%     M*A
%     A*A';
end