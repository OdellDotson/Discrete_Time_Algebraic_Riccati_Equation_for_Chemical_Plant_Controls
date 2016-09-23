function K_Reward = K_Distribution(i, t)
    K_a = 1 + 0.2*sin(t*0.8);
    K_b = .5;
    K_mu = 5 + -sin(t*0.8)*1.1;
    K_sigma = 1+t*0.5;
    K_Reward = K_a.*exp(-((i-K_mu).^2)./(2*(K_sigma.^2))) -K_b; 
end