mypath = regexprep(userpath,':','');
addpath([mypath '/MCMC-SLAM/BaileyUtilities']);
addpath([mypath '/MCMC-SLAM/fastslam']);
addpath([mypath '/MCMC-SLAM/fastslam/fastslam2r']);
addpath([mypath '/MCMC-SLAM/Tools']);

    [lm,wp] = generateCircleMap(100);
    for i=1:100
        [lmErr, stateErr, time]= Fastslam2r_sim(lm, wp, 100);
        
        file = fopen([mypath '/MCMC-SLAM/RunXXX/FastSLAM2_run2_p100'],'a');
        fprintf(file,'Runnning on generateCircleMap(100)\n')
        fprintf(file,'Particle No : 100\n')
        fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
        fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
        fprintf(file,'Elapsed time : %8.6G seconds\n', time)
        fclose(file)
    end
    for i=1:100
        [lmErr, stateErr, time]= Fastslam2r_sim(lm, wp, 1000);
        
        file = fopen([mypath '/MCMC-SLAM/RunXXX/FastSLAM2_run2_p1000'],'a');
        fprintf(file,'Runnning on generateCircleMap(100)\n')
        fprintf(file,'Particle No : 1000\n')
        fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
        fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
        fprintf(file,'Elapsed time : %8.6G seconds\n', time)
        fclose(file)
    end
    for i=1:30
        [lmErr, stateErr, time]= Fastslam2r_sim(lm, wp, 3500);
        
        file = fopen([mypath '/MCMC-SLAM/RunXXX/FastSLAM2_run2_p3500'],'a');
        fprintf(file,'Runnning on generateCircleMap(100)\n')
        fprintf(file,'Particle No : 3500\n')
        fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
        fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
        fprintf(file,'Elapsed time : %8.6G seconds\n', time)
        fclose(file)
    end
    for i=1:30
        [lmErr, stateErr, time]= Fastslam2r_sim(lm, wp, 10000);
        
        file = fopen([mypath '/MCMC-SLAM/RunXXX/FastSLAM2_run2_p10000'],'a');
        fprintf(file,'Runnning on generateCircleMap(100)\n')
        fprintf(file,'Particle No : 10000\n')
        fprintf(file,'landmark average sqL2 error : %8.6G\n', lmErr)
        fprintf(file,'state average sqL2 error : %8.6G\n', stateErr)
        fprintf(file,'Elapsed time : %8.6G seconds\n', time)
        fclose(file)
    end



