function MCMCSLAM_run2()
   [lm,wp] = generateCircleMap(50);

   for i=1:30
        [lmErr, stateErr, time]= MCMCslam_sim_v2(lm, wp, 150, 450);
        
        file = fopen('c:/Works/Papers/MCMC-SLAM/Runnings/MCMC-SLAM_run2b','a')
        fprintf(file,'Runnning on generateCircleMap(50)\n')
        fprintf(file,'MCMC iteration per step : 150\n')
        fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
        fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
        fprintf(file,'Elapsed time : %8.6G seconds\n', time)
        fclose(file)
    end

   for i=1:30
        [lmErr, stateErr, time]= MCMCslam_sim_v2(lm, wp, 200, 450);
        
        file = fopen('c:/Works/Papers/MCMC-SLAM/Runnings/MCMC-SLAM_run2b','a')
        fprintf(file,'Runnning on generateCircleMap(50)\n')
        fprintf(file,'MCMC iteration per step : 200\n')
        fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
        fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
        fprintf(file,'Elapsed time : %8.6G seconds\n', time)
        fclose(file)
   end

   %    for i=1:100
%         [lmErr, stateErr, time]= MCMCslam_sim_v2(lm, wp, 1, 450);
%         
%         file = fopen('c:/Works/Papers/MCMC-SLAM/Runnings/MCMC-SLAM_run2','a')
%         fprintf(file,'Runnning on generateCircleMap(50)\n')
%         fprintf(file,'MCMC iteration per step : 1\n')
%         fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
%         fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
%         fprintf(file,'Elapsed time : %8.6G seconds\n', time)
%         fclose(file)
%     end
%     for i=1:100
%         [lmErr, stateErr, time]= MCMCslam_sim_v2(lm, wp, 10, 450);
%         
%         file = fopen('c:/Works/Papers/MCMC-SLAM/Runnings/MCMC-SLAM_run2','a')
%         fprintf(file,'Runnning on generateCircleMap(50)\n')
%         fprintf(file,'MCMC iteration per step : 10\n')
%         fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
%         fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
%         fprintf(file,'Elapsed time : %8.6G seconds\n', time)
%         fclose(file)
%     end
%     for i=1:10
%         [lmErr, stateErr, time]= MCMCslam_sim_v2(lm, wp, 100, 450);
%         
%         file = fopen('c:/Works/Papers/MCMC-SLAM/Runnings/MCMC-SLAM_run2','a')
%         fprintf(file,'Runnning on generateCircleMap(50)\n')
%         fprintf(file,'MCMC iteration per step : 100\n')
%         fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
%         fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
%         fprintf(file,'Elapsed time : %8.6G seconds\n', time)
%         fclose(file)
%     end
end