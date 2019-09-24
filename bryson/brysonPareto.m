%% Pareto optimal solution for Bryson-Denham
bdSystem = YopSystem(...
    'states', 2, ...
    'controls', 1, ...
    'model', @trolleyModel ...
    );

time = bdSystem.t;
trolley = bdSystem.y;

ocp = YopOcp();
ocp.min({ timeIntegral( 1/2*trolley.acceleration^2 ) });
ocp.st(...
    'systems', bdSystem, ...
    ... % Initial conditions
    {  0  '==' t_0( trolley.position ) }, ...
    {  1  '==' t_0( trolley.speed    ) }, ...
    ... % Terminal conditions
    {  0  '<=' t_f( time ) }, ...
    {  0  '==' t_f( trolley.position ) }, ...
    { -1  '==' t_f( trolley.speed    ) } ...
    );


%% Getting the pareto otimal solution using YopSolve
% This is just to show how long it would take if the problem was solved
% every time with YopSolve 
% Time point vector, with 5 time points
pareto_time_points1 = 0.1:0.2:1;
tic 
for i = 1:length(pareto_time_points1)
    ocp.solve('controlIntervals', 40);
end
yopsolve_time = toc


%% Getting the Pareto otimal solution by reparametrization
% Time point vector, with 20 time points
pareto_time_points2 = 0.1:0.1:2;
tic
% Solving the OCP
ocp.build('controlIntervals', 40);
ocp.parameterize;
for i = 1:length(pareto_time_points2)
    ocp.Independent.setFinalUpper(pareto_time_points2(i));
    ocp.parameterize;
    sol(i)= ocp.optimize;
end
reparam_time = toc

% Objective at time points
sols = [sol.NumericalResults];
J = [sols.Objective];

% Maximum distance from starting point at time points
x = [];
for i = 1:length(pareto_time_points2)
    x(i) = max(sols(i).State(1,:));
end

%% Plot Pareto otimal solution by reparametrization
figure(1)
plot(pareto_time_points2,J)
title('Pareto front')
xlabel('Time')
ylabel('Objective')


figure(2)
plot(x,J)
title('Pareto front')
xlabel('Distance')
ylabel('Objective')


%% Model
function [dx, y] = trolleyModel(time, state, control)

position = state(1);
speed = state(2);
acceleration = control;
dx = [speed; acceleration];

y.position = position;
y.speed = speed;
y.acceleration = acceleration;

end