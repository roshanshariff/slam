mypath = regexprep(userpath,':','');
%mypath = '/usr/eureka/szepesva/';
addpath([mypath '/MCMC-SLAM/BaileyUtilities']);
addpath([mypath '/MCMC-SLAM/MCMC_SLAM_v2']);
addpath([mypath '/MCMC-SLAM/Tools']);
   [lm,wp] = generateCircleMap(200);
    for i=1:20
        [lmErr, stateErr, time]= MCMCslam_sim_v2(lm, wp, 100, 2000);
        
        file = fopen([mypath '/MCMC-SLAM/RunXXX/MCMC-SLAM_run100_1'],'a');
        fprintf(file,'Runnning on generateCircleMap(200)\n')
        fprintf(file,'MCMC iteration per step : 100\n')
        fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
        fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
        fprintf(file,'Elapsed time : %8.6G seconds\n', time)
        fclose(file)
    end

