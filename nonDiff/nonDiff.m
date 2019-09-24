%% Nondifferentiable system
sys = YopSystem(...
    'states', 3, ...
    'controls', 1, ...
    'model', @nonDiffModel);
% Symbolic variables
t = sys.t;
x = sys.x;
u = sys.u;

% Phase description
ocp = YopOcp();
ocp.min({ t_f( x(3) ) }); 
ocp.st(...
     'systems', sys, ...
     ... % Initial conditions
    { 0 '==' t_0(x(1)) }, ...
    { 0 '==' t_0(x(2)) }, ...
    { 0 '==' t_0(x(3)) }, ...
    ... % Final conditions
    { 2 '==' t_f( t )  } ...
    );

w0 = YopInitialGuess(...
    'signals', [t; x; u], ...
    'signalValues', [0; 0; 0; 0; -5] ...
    );

sol = ocp.solve(...
    'controlIntervals', 100, ...
    'collocationPoints', 'radau', ...
    'polynomialDegree', 3, ...
    'initialGuess', w0);

%% Plot the results 
figure(1)
subplot(2,1,1)
sol.plot(t,x);
legend('x1','x2','x3');
title('Nondiff System state variable');

subplot(2,1,2)
sol.plot(t,u);
legend('u');
title('Nondiff System control');

%% Model
function dx = nonDiffModel(t, x, u)
% Nondifferentiable part
d = 100*(if_else(t > 0.5,1,0)-if_else(t > 0.6,1,0));

dx1 = x(2);
dx2 = -x(1)-x(2)+u+d;
dx3 = 5*x(1)^2+2.5*x(2)^2+0.5*u^2;
dx = [dx1;dx2;dx3];
end