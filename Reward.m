function dH = Reward(x, u, t)
    %% From the control input, determine how this changes internal states
    Mg_in = 2*u(1);
    K_in = u(1) + u(2);
    S_in = u(2);
    F_in = u(1) * 3 + u(2) * 2;

    %% Calculating the element rewards for the given time and given control input
    Mg_R = Mg_Distribution(Mg_in, t);
    K_R= K_Distribution(K_in, t);
    S_R = S_Distribution(S_in, t);
    F_R = 0.1*sin(F_in);

    dH = Mg_R + K_R + S_R + F_R;
end