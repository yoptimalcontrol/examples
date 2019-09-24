%% Greenhouse Climate Control, with external input
sys = YopSystem('states', 2, 'controls', 1, ...
    'model', @greenhouseModel);
% Symbolic variables
t = sys.t;
x = sys.x;
u = sys.u;

%% Formulate optimal control problem
p4 = 4.55e-4; 
p5 = 136.4;
tf = 48;

ocp = YopOcp();
ocp.min({ t_f(-p5*x(1)) '+' timeIntegral(p4*u) }); 
ocp.st(...
     'systems', sys, ...
     ... % Initial conditions
    { 0  '==' t_0(x(1)) }, ...
    { 10 '==' t_0(x(2)) }, ...
    ... % Constraints
    { 0  '<=' u '<=' 10 }, ...
    ... % Final conditions
    { tf '==' t_f(t)    } ...
    );
sol = ocp.solve('controlIntervals', 100);

%% Plot
% States, time and control
x1 = sol.signal(sys.x(1))';
x2 = sol.signal(sys.x(2))';
u = sol.signal(sys.u)';
t = sol.signal(sys.t)';

% External input for plots, comes from greenhouseModel
te = sys.y.te;
I  = sys.y.I;
T0 = sys.y.T0;

% Plot external inputs and control
figure(1);
plot(te,I./40,te,T0,t,u); axis([0 tf -1 30]);
xlabel('Time [h]');
ylabel('Heat input, temperatures & light');
legend('Light [W]','Outside temp. [oC]','Heat input [W]');
title('Optimal heating, outside temperature and light');

% Plot the optimal states
figure(2)
sf1=1200; sf3=60;
x3 = cumtrapz(t,p4*u); % Integral(pHc*u)
plot(t,[sf1*x1 x2 sf3*x3]); axis([0 tf -5 30]);
xlabel('Time [h]'); ylabel('states');
legend('1200*Dry weight [kg]','Greenhouse temp. [oC]','60*Integral(pHc*u dt) [J]');
title('Optimal system behavior and the running costs');

%% Model
function [dx, y] = greenhouseModel(t, x, u)
% Constants
p1 = 7.5e-8; 
p2 = 1; 
p3 = 0.1;
tf = 48;
% External inputs: [time, sunlight, outside temperature]
te  = (-1:0.2:49);
I = max(0, 800*sin(4*pi*te/tf-0.65*pi));
T0 = 15+10*sin(4*pi*te/tf-0.65*pi);

% Extract external inputs from table tue through interpolation
d1 = YopInterpolant(te, I);
d2 = YopInterpolant(te,T0);

dx1 = p1*d1(t)*x(2);
dx2 = p2*(d2(t)-x(2))+p3*u;
dx = [dx1; dx2];
y.te = te;
y.I = I;
y.T0 = T0;
end
