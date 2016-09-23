L = @(x,u,t) -x;
M = @(xT, T) 0;
r = @(xT, T) 0;
h = @(x, u)[ -u(1) ; -u(2) ; u(1)-maxU ; u(2)-maxU];
maxU = 5;
m = 2;

x_0 = 4;

T = 15;
N = 3*60;

u = @(x,t) -K(t)*(x-x_traj(t))+u_traj(t);

f = @(x, u, t)Reward(x, u, t);
f_other = @(t,x)dynamics(x,u,t);

[X,U,t,J] = DSS(L,M,h,r,f,x_0,m,T,N); 

[x_traj,u_traj] = discrete2continuous(X,U, T, N);

[A,B] = linearizeAB(f,x_traj,u_traj);
Q = @(t)eye(1);
S = eye(1);
R = @(t)eye(2); %Number of control inputs
[K,P] = dare(A,B,Q,R,S,T,N);

[t,y] = ode45(@(t,x)dynamics(x,u(x,t),t),[0 T],x_0);
%plot(t,y);

%%
%{
subplot(2,1,1)
plot(t,X)
title('State trajectory')
xlabel('Time', 'FontSize', 20);
ylabel('Fish Health', 'FontSize', 20);

subplot(2,1,2)
plot(t,U)
title('Control input trajectories')
xlabel('Time', 'FontSize', 20);
ylabel('Control inputs', 'FontSize', 20);
%}
%%

%{
subplot(2,1,1)
plot(t,X)
title('State trajectory')
xlabel('Time', 'FontSize', 20);
ylabel('Fish Health', 'FontSize', 20);

subplot(2,1,2)
plot(t,U)
title('Control input trajectories')
xlabel('Time', 'FontSize', 20);
ylabel('Control inputs', 'FontSize', 20);
%}


