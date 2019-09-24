function dx = brysonDenhamModel(t, x, u)
pos = x(1);
vel = x(2);
acc = u;

dx = [vel; acc];
end