function Mg_R = Mg_Reward(n, i)
    Ma_a = 1 + 0.05 *sin(i) ;
    Mg_b = .5;
    Mg_mu = 3+sin(i)*3;
    Mg_sigma = 8+i*0.2;
    Mg_R = Ma_a*exp(-((n-Mg_mu).^2)/(2*(Mg_sigma^2))) -Mg_b; 
end