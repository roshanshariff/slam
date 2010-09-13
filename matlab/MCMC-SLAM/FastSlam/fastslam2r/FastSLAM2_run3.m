function FastSLAM2_run3()
    [lm,wp] = generateCircleMap(200);
    for i=1:100
        [lmErr, stateErr, time]= Fastslam2r_sim(lm, wp, 100);
        
        file = fopen('c:/Works/Papers/MCMC-SLAM/Runnings/FastSLAM2_run3','a')
        fprintf(file,'Runnning on generateCircleMap(200)\n')
        fprintf(file,'Particle No : 100\n')
        fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
        fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
        fprintf(file,'Elapsed time : %8.6G seconds\n', time)
        fclose(file)
    end
    for i=1:100
        [lmErr, stateErr, time]= Fastslam2r_sim(lm, wp, 1000);
        
        file = fopen('c:/Works/Papers/MCMC-SLAM/Runnings/FastSLAM2_run3','a')
        fprintf(file,'Runnning on generateCircleMap(200)\n')
        fprintf(file,'Particle No : 1000\n')
        fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
        fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
        fprintf(file,'Elapsed time : %8.6G seconds\n', time)
        fclose(file)
    end
    for i=1:10
        [lmErr, stateErr, time]= Fastslam2r_sim(lm, wp, 10000);
        
        file = fopen('c:/Works/Papers/MCMC-SLAM/Runnings/FastSLAM2_run3','a')
        fprintf(file,'Runnning on generateCircleMap(200)\n')
        fprintf(file,'Particle No : 10000\n')
        fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
        fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
        fprintf(file,'Elapsed time : %8.6G seconds\n', time)
        fclose(file)
    end
end