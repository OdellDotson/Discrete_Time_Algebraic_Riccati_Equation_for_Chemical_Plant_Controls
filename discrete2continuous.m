function [x_traj,u_traj] = discrete2continuous(discretized_X,discretized_U,T,N)
%DISCRETE2CONTINUOUS turns discretized state and control input
%trajectories into continuous-time trajectory by using piece-wise linear
%   [XT,UT] = DISCRETE2CONTINUOUS(DX,DU,T,N) turns discretized state DX and
%   control input DU trajectories into continous-time state XT and control
%   input UT trajectories. T is the final time of the state & control input
%   trajectory. N is the number discretized points.
%   XT is a function handle in term of time.
%   UT is a function handle in term of time.

x_traj = @(t)piecewiseLinear(t,discretized_X,T,N);
u_traj = @(t)piecewiseLinear(t,discretized_U,T,N);

    function pt = piecewiseLinear(t,discretized_p,T,N)
        for j = 1:length(t)
            pt = zeros(size(discretized_p,1),length(t));
            dt = T/N;
            time_vector = linspace(0,T,N+1);
            num_p = size(discretized_p,1);
            if (t(j)<0)
                pt(:,j) = zeros(num_p,1);
            elseif (t(j)>=T)
                pt(:,j) = discretized_p(:,end);
            else
                idx = floor(t(j)*N/T)+1;
                pt(:,j) = discretized_p(:,idx)+(discretized_p(:,idx+1)-discretized_p(:,idx))*(t(j)-time_vector(idx))/dt;
            end
        end
    end
end