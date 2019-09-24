%% Bryson Denham 
% Create the YOP system
bdSystem = YopSystem(...
    'states', 3, ...
    'controls', 1, ...
    'model', @trolleyModel ...
    );

time = bdSystem.t;
trolley = bdSystem.y;

ocp = YopOcp();
ocp.min({ t_f( trolley.mt ) });
ocp.st(...
    'systems', bdSystem, ...
    ... % Initial conditions
    {  0  '==' t_0( trolley.position  ) }, ...
    {  1  '==' t_0( trolley.speed     ) }, ...
    {  0  '==' t_0( bdSystem.x(3) ) }, ...
    ... % Terminal conditions
    {  1  '==' t_f( time ) }, ...
    {  0  '==' t_f( trolley.position  ) }, ...
    { -1  '==' t_f( trolley.speed     ) }, ...
    ... % Constraints
    { 1/9 '>=' trolley.position         } ...
    );

% Solving the OCP
sol = ocp.solve('controlIntervals', 30);

%% Plot the results
figure(1)
subplot(211); hold on
sol.plot(time, trolley.position)
xlabel('Time')
ylabel('Position')

subplot(212); hold on
sol.plot(time, trolley.speed)
xlabel('Time')
ylabel('Velocity')

figure(2); hold on
sol.plot(time, trolley.acceleration)
xlabel('Time')
ylabel('Acceleration (Control)')


%%
function [dx, y] = trolleyModel(time, state, control)

position = state(1);
speed = state(2);
mt = state(3);
acceleration = control;
dx = [speed; acceleration; 1/2*(acceleration)^2];

y.position = position;
y.speed = speed;
y.acceleration = acceleration;
y.mt = mt;

end