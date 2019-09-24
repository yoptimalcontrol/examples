%% Goddard Rocket, Maximum Ascent
% Create the YOP system
sys = YopSystem('states', 3, 'controls', 1, ...
    'model', @goddardSimpleModel);
% Symbolic variables
t = sys.t;
x = sys.x;
u = sys.u;


%% Formulate optimal control problem
ocp(1) = YopOcp();
ocp(2) = YopOcp();
% Objective function to minimize

% Subject to the constraints
x0 = [0; 1; 1];             % Inital value
x1f_min = [0; 1; 0.6];       % Terminal minimum value
x1f_max = [0; inf; 1];     % Terminal maximum value
ocp(1).min({ t_f(-x(2)) });
ocp(1).st(...
     'systems', sys, ...
    {    0  '<=' t_0(t) '<='   0   }, ...
    {    0  '<=' t_f(t) '<='  inf  }, ...
    {  -inf '<='  x(1)  '<='  inf  }, ...
    {    1  '<='  x(2)  '<='  inf  }, ...
    {   0.6 '<='  x(3)  '<='   1   }, ...
    {   x0  '==' t_0(x)            }, ...
    {x1f_min '<=' t_f(x) '<=' x1f_max}, ...
    {    0  '<='   u    '<='  3.5  } ...
    );

x2f_min = [0; 1; 0.6];       % Terminal minimum value
x2f_max = [0; 1; 0.6];     % Terminal maximum value
ocp(2).min({ t_f( 0 ) });
ocp(2).st(...
     'systems', sys, ...
    {    0  '<=' t_0(t) '<='   inf   }, ...
    {    0  '<=' t_f(t) '<='  inf  }, ...
    {  -inf '<='  x(1)  '<='  inf  }, ...
    {    1  '<='  x(2)  '<='  inf  }, ...
    {   0.6 '<='  x(3)  '<='   1   }, ...
    {x2f_min '<=' t_f(x) '<=' x2f_max}, ...
    {    0  '<='   u    '<='  3.5  } ...
    );

% Solving the OCP
sol = ocp.solve('controlIntervals', 100);

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

%% Rocket animation
animateRocket(sol, 'goddardLandingV2.gif', 6);





