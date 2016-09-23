function K_R = K_Reward(n, i)
    K_a = 1 + 0.2*sin(i*0.8);
    K_b = .5;
    K_mu = 5 + -sin(i*0.8)*1.1;
    K_sigma = 5+i*0.5;
    K_R = K_a*exp(-((n-K_mu).^2)/(2*(K_sigma^2))) -K_b; 
end