function Mg_Reward = Mg_Distribution(i, t)
    Ma_a = 1 + 0.05 *sin(t) ;
    Mg_b = .5;
    Mg_mu = 3+sin(t)*3;
    Mg_sigma = 3+t*0.2;
    if size(Mg_sigma) ~= [1 1],
        Mg_sigma
    end
    Mg_Reward = Ma_a.*exp(-((i-Mg_mu).^2)./(2*(Mg_sigma.^2))) -Mg_b; 
end