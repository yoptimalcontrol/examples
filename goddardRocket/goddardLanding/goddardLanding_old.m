%% Goddard Rocket Landing
% Create the YOP system
sys = YopSystem('states', 3, 'controls', 1, ...
    'model', @goddardRocketModel);
% Symbolic variables
t = sys.t;
x = sys.x;
u = sys.u;

%% Formulate optimal control problem
ocp(1) = YopOcp();
ocp(2) = YopOcp();

% Constants
m0 = 214.839;
mf = 67.9833;
Fm = 9.525515;

% Phase 1
ocp(1).min({ t_f( -x(2) ) });
x0 = [0; 0; m0];
xf1_min = [0; 0; mf];
xf1_max = [0; inf; m0];
ocp(1).st(...
     'systems', sys, ...
    {    0   '<=' t_0(t) '<='  0     }, ...
    {    0   '<=' t_f(t) '<=' inf    }, ...
    {    0   '<='  x(1)  '<=' inf    }, ...
    {    0   '<='  x(2)  '<=' inf    }, ...
    {   mf   '<='  x(3)  '<='  m0    }, ...
    {   x0   '==' t_0(x)             }, ...
    {xf1_min '<=' t_f(x) '<=' xf1_max}, ...
    {    0   '<='   u    '<=' Fm     } ...
    );

% Phase 2
ocp(2).min({ t_f( 0 ) });
xf2 = [0; 0; mf];
ocp(2).st(...
     'systems', sys, ...
    {   0  '<=' t_0(t) '<=' inf }, ...
    {   0  '<=' t_f(t) '<=' inf }, ...
    { -inf '<='  x(1)  '<=' inf }, ...
    {   0  '<='  x(2)  '<=' inf }, ...
    {  mf  '<='  x(3)  '<=' m0  }, ...
    {  xf2 '==' t_f(x)          }, ...
    {   0  '<='   u    '<=' Fm  } ...
    );

% Solving the OCP
sol = ocp.solve('controlIntervals', 100, 'polynomialDegree', 3);

%% Plot the results 
figure(1)
subplot(311); hold on
sol(1).plot(sys.t, sys.x(1))
sol(2).plot(sys.t, sys.x(1))
xlabel('Time')
ylabel('Velocity')

subplot(312); hold on
sol(1).plot(sys.t, sys.x(2))
sol(2).plot(sys.t, sys.x(2))
xlabel('Time')
ylabel('Height')

subplot(313); hold on
sol(1).plot(sys.t, sys.x(3))
sol(2).plot(sys.t, sys.x(3))
xlabel('Time')
ylabel('Mass')

figure(2); hold on
sol(1).stairs(sys.t, sys.u)
sol(2).stairs(sys.t, sys.u)
xlabel('Time')
ylabel('F (Control)')

figure(3); hold on
sol(1).plot3(sys.x(2) , sys.x(1), sys.y.Fa, 'LineWidth',2) 
sol(2).plot3(sys.x(2) , sys.x(1), sys.y.Fa, 'LineWidth',2) 
xlabel('h')
ylabel('v')
zlabel('Fa')
grid minor

%% Rocket animation
animateRocket(sol, 'goddardLanding2.gif', 6);