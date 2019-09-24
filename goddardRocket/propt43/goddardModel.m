function dx = goddardModel(t, x, u)
% Model
v = x(1);
h = x(2);
m = x(3);
T = u;

c = 0.5;
g0 = 1;
h0 = 1;
D0 = 0.5*620;
b = 500;
g = 1/h^2;
% D = 0.5*620*v^2*exp(-b*(h-h0)/h0);
D = D0*v^2*exp(-b*h);

dv = (T-sign(v)*D)/m-g;
dh = v;
dm = -T/c;

dx = [dv;dh;dm];
end