function [K,P] = dare(A,B,Q,R,S,T,N)
%DARE solve a given formulation of a differential algebraic riccati
%equation
%   [K,P] = DARE(A,B,Q,R,S) solves d.a.r.e. where A,B,Q,R,S are function handles in
%   term of time. The function returns a function handle of Gain matrix K (time dependent)
%   and P-matrix. 

n = size(A(0),2);
P_state_final = reshape(S,n^2,1);
u = @(P_state,t)0;

% backward simulation of DARE
[P_state,~,discrete_t] = dynSim(@(P_state,u,t)dareFun(P_state,u,t,A,B,Q,R),u,P_state_final,T,-T/N);

% Collocation for Quartic Polynomial approximation of P(t)

dt = T/N;
In = eye(n^2);
On = zeros(n^2);
C = [In  On    On        On;...
    In   In    In        In ;...
    On   In/dt On        On;...
    On   In/dt 2*In/dt^2 3*In/dt^3];
D = [   P_state(:,1:end-1)   ;...
        P_state(:,2:end)     ;...
        dareFun(P_state(:,1:end-1),u,discrete_t(1,1:end-1),A,B,Q,R);...
        dareFun(P_state(:,2:end),u,discrete_t(1,2:end),A,B,Q,R)];

w = C\D;
w = reshape(w,4*size(A(discrete_t),2)^2,N); % cubic polynomial
n = size(A(0),2);

% set up function handles
    function K = Kt(t,R,B,discrete_t,w,n,T,N,dt)
        [K,~] = P_matrix(t,R,B,discrete_t,w,n,T,N,dt);
    end
    function P = Pt(t,R,B,discrete_t,w,n,T,N,dt)
        [~,P] = P_matrix(t,R,B,discrete_t,w,n,T,N,dt);
    end

K = @(t)Kt(t,R,B,discrete_t,w,n,T,N,dt);
P = @(t)Pt(t,R,B,discrete_t,w,n,T,N,dt);

K = @(t)K(t);
P = @(t)P(t);

    function [K,P] = P_matrix(t,R,B,discrete_t,w,n,T,N,dt)
        w = reshape(w,4*n^2,N);
        P = zeros(n,n,length(t));
        K = zeros(size(R(1),1),n,length(t));
        for i = 1:length(t),
            if(t(i)<=0)
                P_temp = [eye(n*n) eye(n*n)*(0/dt) eye(n*n)*(0/dt)^2 eye(n*n)*(0/dt)^3]*w(:,1);
            elseif (t(i)>=T)
                P_temp = [eye(n*n) eye(n*n)*(dt/dt) eye(n*n)*(dt/dt)^2 eye(n*n)*(dt/dt)^3 ]*w(:,end);
            else
                idx = floor(t(i)/dt)+1;
                s = (t(i)-discrete_t(idx))/dt;
                P_temp = [eye(n*n) eye(n*n)*s eye(n*n)*(s)^2 eye(n*n)*(s)^3 ]*w(:,idx);
            end
            
            P_temp = reshape(P_temp,n,n);
            P(:,:,i) = P_temp;
            K(:,:,i) = R(t(i))\B(t(i))'*P_temp;
        end
    end
end

function dP_state = dareFun(P_state,~,t,A,B,Q,R)
At = zeros(size(A(1),1),size(A(1),2),length(t));
Bt = zeros(size(B(1),1),size(B(1),2),length(t));
Qt = zeros(size(Q(1),1),size(Q(1),2),length(t));
inRt = zeros(size(R(1),1),size(R(1),2),length(t));
for i = 1:length(t)
    At(:,:,i) = A(t(i));
    Bt(:,:,i) = B(t(i));
    Qt(:,:,i) = Q(t(i));
    inRt(:,:,i) = inv(R(t(i))); 
end

P = reshape(P_state,size(A(1),2),size(A(1),2),size(P_state,2));
temp = mmat(Bt,mmat(inRt,permute(Bt,[2 1 3])));
dP = -(mmat(permute(At,[2 1 3]),P)+mmat(P,At)-mmat(P,mmat(temp,P))+Qt);
dP_state = reshape(dP,size(P_state,1),size(P_state,2));
end


