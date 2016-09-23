function S_Reward = S_Distribution(i, t)
    S_a = 1+ 0.1*sin(t);
    S_b = .5;
    S_mu = 7 + sin(t*1.2)*0.8;
    S_sigma = 5+t*0.1;
    S_Reward = S_a.*exp(-((i-S_mu).^2)./(2*(S_sigma.^2))) -S_b; 
end

