mypath = regexprep(userpath,':','');
%mypath = '/usr/eureka/szepesva/';
addpath([mypath '/MCMC-SLAM/BaileyUtilities']);
addpath([mypath '/MCMC-SLAM/fastslam']);
addpath([mypath '/MCMC-SLAM/fastslam/fastslam2r']);
addpath([mypath '/MCMC-SLAM/Tools']);
    [lm,wp] = generateCircleMap(200);
    for i=1:20
        [lmErr, stateErr, time]= Fastslam2r_sim(lm, wp, 10000, 2000);
        
        file = fopen([mypath '/MCMC-SLAM/RunXXX/FastSLAM2_run2_p10000_1'],'a');
        fprintf(file,'Runnning on generateCircleMap(200)\n')
        fprintf(file,'Particle No : 10000\n')
        fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
        fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
        fprintf(file,'Elapsed time : %8.6G seconds\n', time)
        fclose(file)
    end
