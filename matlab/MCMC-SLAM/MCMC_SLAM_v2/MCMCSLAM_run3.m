function MCMCSLAM_run3()
   [lm,wp] = generateCircleMap(200);
%     for i=1:100
%         [lmErr, stateErr, time]= MCMCslam_sim_v2(lm, wp, 1, 2000);
%         
%         file = fopen('c:/Works/Papers/MCMC-SLAM/Runnings/MCMC-SLAM_run3','a')
%         fprintf(file,'Runnning on generateCircleMap(200)\n')
%         fprintf(file,'MCMC iteration per step : 1\n')
%         fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
%         fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
%         fprintf(file,'Elapsed time : %8.6G seconds\n', time)
%         fclose(file)
%     end
    for i=1:100
        [lmErr, stateErr, time]= MCMCslam_sim_v2(lm, wp, 10, 2000);
        
        file = fopen('c:/Works/Papers/MCMC-SLAM/Runnings/MCMC-SLAM_run3','a')
        fprintf(file,'Runnning on generateCircleMap(200)\n')
        fprintf(file,'MCMC iteration per step : 10\n')
        fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
        fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
        fprintf(file,'Elapsed time : %8.6G seconds\n', time)
        fclose(file)
    end
    for i=1:10
        [lmErr, stateErr, time]= MCMCslam_sim_v2(lm, wp, 100, 2000);
        
        file = fopen('c:/Works/Papers/MCMC-SLAM/Runnings/MCMC-SLAM_run3','a')
        fprintf(file,'Runnning on generateCircleMap(200)\n')
        fprintf(file,'MCMC iteration per step : 100\n')
        fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
        fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
        fprintf(file,'Elapsed time : %8.6G seconds\n', time)
        fclose(file)
    end
end