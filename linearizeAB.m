function [A,B] = linearizeAB(f,x_traj,u_traj)
%LINEARIZEAB linearize a given dynamic system and return state matrix and
%input matrix
%   [A,B] = LINEARIZEAB(F,XT,UT) linearizes a given dynamic F to get
%   time-dependent linearized state matrix A and input matrix B.
%   A and B are function handles in term of time t.

n = size(x_traj(1),1);
m = size(u_traj(1),1);
x = sym('x',[n,1]);
u = sym('u',[m,1]);

A = @(t)linearize(t,x,f,x,u,x_traj,u_traj);
B = @(t)linearize(t,u,f,x,u,x_traj,u_traj);
A = @(t)A(t);
B = @(t)B(t);

    function M = linearize(t,w,f,x,u,x_traj,u_traj)
        M = zeros(size(x,1),numel(w),length(t));
        for i = 1:length(t)
            M_sym = jacobian(f(x,u,t(i)),w);
            M_sym = subs(M_sym,x,x_traj(t(i)));
            M(:,:,i) = eval(subs(M_sym,u,u_traj(t(i))));
        end
    end
end