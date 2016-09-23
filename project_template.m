function [t,x]= project_template
%% Description, Authors, etc 
%{
    Write a brief description of your project. 
    -   What is your system ? 

    The system modeled in this project is an autonomous robotic fish tank. 
    The fish require a certain level of food as well as a balance of 
    magnesium, potasium and sodium. 

    -   What the state variables ? 
    
    The state of the system is defined by the amount of food, the level of
    the three elemented mentioned before, (and the happiness of the fish.)?

    -   How do you control the system ? How many control inputs do you have ?

    The numer of copeopods and phytoplankton released into the system as
    food is controlled autonomously by the fish tank. This means that the
    system has two control inputs: C and P. Each is simply a single scaler value.

    -   What the goal of the control? What's the cost function ?

    The purpose of the control is to prevent the fish from dying.
    Maximizing the reward from each of the chemicals and food level as
    described below is the cost function. (Actually, it is minimizing the
    negative reward.)

    -   What kind of process do you use to come upwith control policy?

    The needs for each of the chemicals is a dynamically shifting gausian
    distribution. This gives each chemical level a "sweet spot", where too
    much or to little of the chemical results in much less reward. Keeping
    the chemicals at a healthy level in a fish tank is one of the more
    challenging requirements for a fish tank maintainer. These dynamic
    gausian distributions simulate that difficulty. 

    Odell Dotson 
%}
%  EXAMPLE for Description
%{
    This is a project template for "Numerical Optimal Control".

    System Description:
    This example uses a basic single degree-of-freedom spring-mass-damper
    system. There are 2 state variable, position of the block and velocity
    of the block. We assume that we can exert control force directly on the
    block. And there is only one control input.

    Problem Statement:
    The goal is to move the block to the desire position (5 meter from the
    wall). The block starts at 10 meter from the wall. The quadratic cost
    function is used to minimize the control effort.

    Control Methodology:
    Standard Linear Quadratic Regulator (LQR) is used to come up with a
    gain matrix K, which later is used for full state-feedback.

    Author: Pi Thanacha Choopojcharoen
%}
%% Parameters
%{
    This is where you assign all parameters in the system such as mass,
    length, spring constant.

    Use assignParameter to create a struct of parameters. Go to the function
    declaration and assign each parameter as seen in example
%}
m = 1;
b = 0.1;
k = 10;
h = 1;
parameter = assignParameter(m,b,k,h); % m,b,k,h are for examples

%% Control Policy
%{
    Your control policy should be defined inside the function declaration
    of controlAnalysis. Notice, the control policy is a function that
    depends on state variables (x) and time (t).
%}
u = @(x,t)controlAnalysis(x,t,parameter);

%% Dynamic System
%{
    Your dynamic system should be defined inside function declaration of
    dynamics. You have to transofrm your dynamic system into state-space
    representation.
%}
f = @(t,x)dynamics(x,u,t,parameter);

%% Simulation
%{
    This is where you set up your simulation parameter, such as final time
    (T), initial states (x_0). Note: final time might be determined from
    your control analysis if you have a problem with free final time.
%}
T = 5;
x_0 = [10;0];
[t,x] = ode45(f,[0 T],x_0);

%% visualization
%{
    You should define how to visualize your result inside a function
    visualization.
%}
visualization(x,u,t,parameter);

end

%% Dynamic Related Functions

function parameter = assignParameter(m,b,k,h)
%{
    You have to change each field of the struct according to your porject.
    This allows us to pass parameters easily to other functions.
%}
parameter.m = m;
parameter.b = b;
parameter.k = k;
parameter.h = h;

end
function [m,b,k,h] = getParameter(parameter)
%{
    In addition to assignParameter, many functions require you to extract
    some specific parameters from the struct. Change this code, so that the
    parameters match with your project.
%}

m = parameter.m;
b = parameter.b;
k = parameter.k;
h = parameter.h;

end
function dx = dynamics(x,u,t,parameter)
%{
    This is where you define your dynamic system. Make sure to include
    getParameter. Notice, u is a function of x and t.
%}
[m,b,k,~] = getParameter(parameter);
dx = [0 1 ; -k/m -b/m]*x+[0;1/m]*u(x,t);
end

%% Control Analysis and Contruction Functions
function control = controlAnalysis(x,t,parameter)
%{
    This is where you put most of your control design algorithm. You might
    need other functions, sub-fuctions, or nested functions.

    The example uses LQR.
%}

[m,b,k,~] = getParameter(parameter);

% desire position
d_desire = 5;

% Linearized Full-state Feedback
A = [0 1; -k/m -b/m];
B = [0;1];
Q = diag([10,10]);
R = 1;
N = zeros(2,1);
[K,~,~] = lqr(A,B,Q,R,N);

control = k*d_desire+K*(bsxfun(@minus,[d_desire;0],zeros(2,numel(t)))-x);

end

%% Visualization Functions

function visualization(x,u,t,parameter)
%{ 
    You might not need the animation for the simulation. But you definitely
    need plot for the final result.

    If you want to create an animation, you have to define a drawSystem
    function, which will be called in each iteration of for-loop.
%}

%% Animation
%{
    This is where the animation loop is defined. If you want an animation
    in your project, please keep this part. Otherwise, remove the for-loop.

    The actual drawing has to be defined inside drawSystem.
%}
step = 1;
figure(1)
for i = 1:step:length(t)-step,
    drawSystem(x(i,:)',parameter);
    hold off;
    pause(t(i+step)-t(i));
end

%% Plots
%{
    This is where you plots your results. Each result will be plotted
    against time (t). Make sure to plot each states individually.
    If you use indirect method, be sure to plot the costate variables.
    The most important part is to plot the control input trajectory.
%}
figure(2)
subplot(2,2,1)
plot(t,x(:,1))
xlabel('t')
ylabel('x')
subplot(2,2,3)
plot(t,x(:,2))
xlabel('t')
ylabel('v')

subplot(2,2,[2 4])
plot(t,u(x',t))
xlabel('t')
ylabel('u')
end
function hp = drawSystem(x,parameter)
%{
    drawSystem draws a picture of a system at a given snapshot of states (x)
%}
[~,~,~,h] = getParameter(parameter);
hp = cell(1,5);
d = x(1);
n = 10;
spring_x = 0:0.01:(d-h/2);
spring_y = 0.5*sin(2*pi*n*spring_x/(d-h/2))+h/2;
hp{1} = plot(spring_x,spring_y,'b','linewidth',2);
hold on;
vertices = [d-h/2 d+h/2 d+h/2 d-h/2 d-h/2;...
            0     0     h     h     0];
hp{2} = plot(vertices(1,1:2),vertices(2,1:2),'r','linewidth',2);
hp{3} = plot(vertices(1,2:3),vertices(2,2:3),'r','linewidth',2);
hp{4} = plot(vertices(1,3:4),vertices(2,3:4),'r','linewidth',2);
hp{5} = plot(vertices(1,4:5),vertices(2,4:5),'r','linewidth',2);
hp{6} = plot([-1 10],[0 0],'k');
hp{7} = plot([0 0],[0 5],'k');
axis equal;
xlabel('x [m]')
ylabel('y [m]')
title('control the block to reach d = 5 m')
end
