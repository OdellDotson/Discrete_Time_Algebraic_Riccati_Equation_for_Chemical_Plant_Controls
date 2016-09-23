function [xTraj,uTraj,tVector] = dynSim(f,u,x_initial,T,dt)
%DYNSIM numerically simulates a given controlled dynamical system from a given
%initial/final condition
%   [XT,UT,TV] = DYNSIM(F,U,XI,T,DT)numerically simulates a following
%   dynamical system
%
%   dx(t)/dt = f(t,x(t),u(x(t),t)) : t = [0,T] : x(0) = XI or x(T) = XI
%
%   where f(x,@(x,t)u,t) is a function F that represents the dynamic of a system.
%         u(x,t) is a function U that represents a given c9ontrol policy
%         T is the final time and DT is the step-size for numeric simulation
%         If DT > 0, DYNSIM will simulate the system forward in time
%         starting at t = 0
%         If DT < 0 DYNSIM will simulate the system backward in time
%         starting at t = T
%   The funtion returns a discretized state trajectory XT and discretized
%   control input trajectory UT, as well as time vector TV.
%
%   Example 1: Simulating a controlled single-degree-of-freedom arm with
%              feedback control forward in time from t= [0,5]
%
%       m = 1; l = 1; g = 9.81; b = 1; xd = pi/4;
%       f = @(t,x,u)[x(2);(m*g*l*sin(x(1))-b*x(2)+u)/(m*l^2)];
%       K = [10 3];
%       u = @(x,t)-K*[x(1,:)-xd;x(2,:)]-m*g*l*sin(xd);
%       [X,U,t] = dynSim(f,u,[pi/2 ; 0],5,0.1);
%       subplot(2,1,1); plot(t,X); xlabel('t');ylabel('x');
%       subplot(2,1,2); plot(t,U); xlabel('t');ylabel('u');
%
%   Example 2: Simulating an uncontrolled mass-spring-damper system 
%              backward in time from t = [10,0]
%
%       m = 1;b = 0.1; k = 1;
%       f = @(t,x,u)[0 1 ; -k/m -b/m]*x;
%       u = @(x,t)0;
%       [X,~,t] = dynSim(f,u,[1 ; 0],10,-0.1);
%       plot(t,X);
%

%% 
% The numerical integration method has a fixed step-size, 
% simulate up to the nearest T

N = floor(abs(T/dt));           
T_f = norm(dt)*N;
tVector = (T_f-sign(dt)*T_f)/2:dt:(T_f+sign(dt)*T_f)/2; 
[tVector,xTraj] = ode45(@(t,x)f(t,x,u(x,t)),tVector,x_initial);
tVector = tVector';
xTraj = xTraj';
if dt<0
    xTraj = fliplr(xTraj);
    tVector = fliplr(tVector);
end
uTraj = zeros(size(u(xTraj(:,1),tVector(1)),1),N+1);
for i = 1:length(tVector)
uTraj(:,i) = u(xTraj(:,i),tVector(i));
end
end