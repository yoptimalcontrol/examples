function dx = goddardRocketFixedTfModel(t, x, u)
% Model
v = x(1);
h = x(2);
m = x(3);
F = u;

D0 = 0.01227; beta = 0.145e-3;
c  = 2060;    
g0 = 9.81;
r0 = 6.371e6; 
r02=r0*r0;
D = D0*v.^2.*exp(-beta*h);
g = g0*r02./(r0+h).^2;

dv = (F*c-sign(v)*D-g*m)/m;
dh = v;
dm = -F;
dx = [dv;dh;dm];
end