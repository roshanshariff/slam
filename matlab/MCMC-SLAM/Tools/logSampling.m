function ind = logSampling( density )
    sumD = 0;
    for i = 1:size(density,2)
        sumD = sumD + density(i);
    end
    
    r = rand(1,1)*sumD;
    
    ind = 1;
    sd = density(1);
    while( sd < r )
        ind = ind + 1;
        sd = sd + density(ind);
    end
end