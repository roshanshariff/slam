function data= MCMCslam_sim(lm, wp)
%function data= MCMCslam_sim(lm, wp)
%
% INPUTS: 
%   lm - set of landmarks
%   wp - set of waypoints
%
% OUTPUTS:
%   data - set of particles representing final state
%
% NOTES:
%   This program is an MCMC-SLAM simulator. To use, create a set of landmarks and 
%   vehicle waypoints (ie, waypoints for the desired vehicle path). The program
%   'frontend.m' may be used to create this simulated environment - type
%   'help frontend' for more information.
%       The configuration of the simulator is managed by the script file
%   'configfile.m'. To alter the parameters of the vehicle, sensors, etc
%   adjust this file. There are also several switches that control certain
%   filter options.
%
% Peter Torma 2008.
% Version 1.0

format compact
path(path, '../')
configfile;

wp = [[0;0], wp];

h= setup_animations(lm,wp);
veh= [0 -WHEELBASE -WHEELBASE; 0 -1 1];
plines=[];

% initialisations
xtrue= zeros(3,1);

dt= DT_CONTROLS; % change in time between predicts
dtsum= 0; % change in time since last observation
ftag= 1:size(lm,2); % identifier for each landmark
da_table= zeros(1,size(lm,2)); % data association table 
data= initialise_store(xtrue,xtrue); % stored data for off-line
iwp= 1; % index to first waypoint 
G= 0; % initial steer angle
G_ind =0;
GVec = [];
zVec = [];
stateIndex = 1;
trajectoryState = xtrue;
mapState = [];
trueFeatureCoordinates = [];
pcount=0;
lmLastSeenTable= zeros(2,size(lm,2)); % 0 if the landmark has not been seen, and holds the last state index where it was seen last 
xfp = [];

if SWITCH_SEED_RANDOM ~= 0, rand('state',SWITCH_SEED_RANDOM), randn('state',SWITCH_SEED_RANDOM), end

Qe= Q; Re= R;
if SWITCH_INFLATE_NOISE==1, Qe= 2*Q; Re= 2*R; end

%if SWITCH_PROFILE, profile on -detail builtin, end

mcmcSLAM = MCMC_SLAM( V, Q, DT_CONTROLS, WHEELBASE, R );

%aviobj = avifile('c:/Works/MatlabProjects/MCMC_SLAM_v2/figures/video/video8.avi', 'compression', 'Cinepak');
%aviobj.quality = 100;


% Main loop 
iterNo = 0;
logLikelihoodHistory = [];
errorHistory = [];
while iwp ~= 0 %&& iterNo < 60
    iterNo = iterNo + 1;
    
    % Compute true data
    [G,iwp]= compute_steering(xtrue, wp, iwp, AT_WAYPOINT, G, RATEG, MAXG, dt);
    if iwp==0 & NUMBER_LOOPS > 1, iwp=1; NUMBER_LOOPS= NUMBER_LOOPS-1; end % path loopfs repeat
    % Add process noise
    [Vn,Gn]= add_control_noise(V,G,Q, SWITCH_CONTROL_NOISE);
    xtrue= predict_true(xtrue, Vn,Gn, WHEELBASE,dt);
    G_ind = [G_ind, G];
    
    % Observe step
    dtsum= dtsum + dt;
    zVec = [];
    if dtsum >= DT_OBSERVE
        dtsum= 0;
        G_ind = [G_ind, stateIndex];
        GVec = [GVec, G_ind.'];    
        G_ind = 0;
               
        % Compute true data, then add noise
        [z,ftag_visible]= get_observations(xtrue, lm, ftag, MAX_RANGE);
        z = add_observation_noise(z,R, SWITCH_SENSOR_NOISE);
        if ~isempty(z)
            plines= make_laser_lines (z,xtrue); 
            for i=1:size(z,2)
                zV = [ stateIndex, z(:,i).', ftag_visible(i) ];
                zVec = [zVec, zV.'];
            end
        end
            
        % Compute (known) data associations
%        [zf,idf,zn,da_table]= data_associate_known(xtrue,z,ftag_visible, da_table);        
%        [trajectoryState, mapState, lmLastSeenTable] = MCMC_build( trajectoryState, mapState, GVec, zVec, lmLastSeenTable );
%        [trajectoryState, mapState] = MCMC_step( trajectoryState, mapState, GVec, zVec );
        if stateIndex == 1
            stateCoordinates   = mcmcSLAM.getState();
            featureCoordinates = mcmcSLAM.getFeatureCoordinates();
        else
            stateCoordinates   = mcmcSLAM.getMLState();
            featureCoordinates = mcmcSLAM.getMLFeatureCoordinates();
        end

%        %Finding the best rotation
%        avgAngle = 0;
%        for i= 1:size(stateCoordinates,2) %stateIndex
%            v1=data.true(1:2,i);
%            v2=stateCoordinates(1:2,i);
%        	avgAngle = avgAngle + atan2(v1(2), v1(1)) - atan2(v2(2), v2(1));
%        end
%        avgAngle = -avgAngle/size(stateCoordinates,2)

        %Finding the best rotation based on landmark positions
        avgAngle = 0;
        for i= 1:size(trueFeatureCoordinates,2)
            v1=trueFeatureCoordinates(1:2,i);
            v2=featureCoordinates(1:2,i);
        	avgAngle = avgAngle + atan2(v1(2), v1(1)) - atan2(v2(2), v2(1));
        end
        avgAngle = avgAngle/size(trueFeatureCoordinates,2);
 
        %rotate it back with the best rotation
        rot = [cos(avgAngle) -sin(avgAngle); sin(avgAngle) cos(avgAngle)];
        stateCoordinates(1:2,:) = rot*stateCoordinates(1:2,:);
        if size(featureCoordinates,2)>1
            featureCoordinates(1:2,:) = rot*featureCoordinates(1:2,:);
        end
        
        %calculating average error
        avgErr=0;
        for i= 1:size(trueFeatureCoordinates,2)
            v1=trueFeatureCoordinates(1:2,i);
            v2=featureCoordinates(1:2,i);
        	avgErr = (v1(2)-v2(2))^2+(v1(1)-v2(1))^2;
        end
        if size(trueFeatureCoordinates,2)>1 
            avgErr = sqrt( avgErr/size(trueFeatureCoordinates,2) );
            errorHistory = [errorHistory,avgErr];
        end
        
        stateIndex = stateIndex + 1;

        
        data=store_data(data, stateCoordinates(:,size(stateCoordinates,2)), xtrue);
        for i = 1:size(stateCoordinates,2)
            s = stateCoordinates(:,i);
            data.path(:,i) = s(1:3);
        end

        xfp = [];
        for i = 1:size(featureCoordinates,2)
            zV  = featureCoordinates(:,i);
            xfp = [xfp, zV];
        end        
    end
    
%    if size(trueFeatureCoordinates,2) > 1
%        figure(1);
%        plot(trueFeatureCoordinates(1,:),trueFeatureCoordinates(2,:),'y*')
%        hold on;
%    end    
        
    if( size(zVec,1) > 0 )
        firstSeenFeatureIdx=[];
        [mcmcSLAM, firstSeenFeatureIdx] = mcmcSLAM.addObservedState( G, zVec );
        trueFeatureCoordinates=[trueFeatureCoordinates, lm(:,firstSeenFeatureIdx)];        
        [mcmcSLAM, llHistory] = mcmcSLAM.step( 10 );
        if( mod( iterNo, 1 ) ==0)
            print( 1, '-djpeg', strcat('c:/Works/MatlabProjects/MCMC_SLAM_v2/figures/video/v', numTOstr(iterNo/1, 5) ) );       
 %           F = getframe(figure(1));
 %           aviobj = addframe(aviobj,F);
            save('c:/Works/MatlabProjects/MCMC_SLAM_v2/figures/logLikelihood.txt','logLikelihoodHistory','-ASCII')
            save('c:/Works/MatlabProjects/MCMC_SLAM_v2/figures/landMarkL2Error.txt','errorHistory','-ASCII')
        end
        if( mod( iterNo, 100 ) ==2)
%            print( 42, '-djpeg', strcat('c:/Works/MatlabProjects/MCMC_SLAM_v2/figures/video/l', numTOstr(iterNo/100, 5) ) ); 
           figure(42);
            %    subplot(1,2,2)
            plot(logLikelihoodHistory);
            figure(43);
            plot(errorHistory);
        end
        logLikelihoodHistory = [logLikelihoodHistory, llHistory];
    else
        mcmcSLAM = mcmcSLAM.addState( G );        
    end
    

    
    
    if dtsum==0
        pcount= pcount+1;
        if pcount == 1 %15
            set(h.pth, 'xdata', data.path(1,1:data.i), 'ydata', data.path(2,1:data.i))    
            pcount=0;
        end
        if ~isempty(z)
            set(h.obs, 'xdata', plines(1,:), 'ydata', plines(2,:))
        end
    end
    if ~isempty(xfp), set(h.xfp, 'xdata', xfp(1,:), 'ydata', xfp(2,:)), end

    
    % Plots
    stateLinkLogLikelihood        = mcmcSLAM.getStateLinkLogLikelihood();
    stateFeatureLinkLogLikelihood = mcmcSLAM.getStateFeatureLinkLogLikelihood();
    do_plot( h, xtrue, plines, veh );
        
%    figure(43);
%    plot(stateLinkLogLikelihood, 'g');

%    figure(44);
%    plot(stateFeatureLinkLogLikelihood, 'r');
end
%aviobj = close(aviobj);
%drawing debug info
%mcmcSLAM.drawFullGraph(lm, wp);
%mcmcSLAM.drawChangingLinks( 10 );
%mcmcSLAM.drawChangingLinks( 2 );
%mcmcSLAM.drawChangingLinks( 3 );

%mcmcSLAM.drawMove( lm, wp );
%mcmcSLAM.drawMove( lm, wp );

%if SWITCH_PROFILE, profile report, end

% 
%


function s = numTOstr(n, nrOfChars)
%
% returns a nrOfChars long string of 'n' extended with 0s if needed

s = num2str(n);

d  = nrOfChars - length(s);

if d < 0
   s = s(1:nrOfChars);
else
   for i=1:d
       s = strcat(num2str(0),s);
   end
end

function h= setup_animations(lm,wp)
figure(1);
%xlim([-112 110])
%ylim([-100 100])
%subplot(1,2,1)
plot(lm(1,:),lm(2,:),'k*')
axis([-125 110 -100 100])
xlim([-125 110])
ylim([-100 100])
hold on, %axis equal
%plot(wp(1,:),wp(2,:), wp(1,:),wp(2,:),'ro', 'LineWidth', 2)
plot(wp(1,:),wp(2,:), wp(1,:),wp(2,:),'b', 'LineWidth', 2)

h.xt= patch(0,0,'g','LineWidth',3,'erasemode','xor'); % vehicle true
h.pth= plot(0,0,'k','LineWidth',5,'erasemode','background'); % vehicle path estimate
%h.xm= patch(0,0,'r','erasemode','xor'); % mean vehicle estimate
h.obs= plot(0,0,'m','LineWidth',3,'erasemode','xor'); % observations
h.xfp= plot(0,0,'ro','LineWidth',2,'markersize', 6, 'erasemode','background'); % estimated features 
%h.xvp= plot(0,0,'r.','erasemode','xor'); % estimated vehicle (particles)
%h.cov= plot(0,0,'erasemode','xor'); % covariances of max weight particle


function do_plot(h, xtrue, plines, veh )

xt= transformtoglobal(veh,xtrue);
set(h.xt, 'xdata', xt(1,:), 'ydata', xt(2,:))
%if ~isempty(plines), set(h.obs, 'xdata', plines(1,:), 'ydata', plines(2,:)), end

drawnow

function p= make_laser_lines (rb,xv)
if isempty(rb), p=[]; return, end
len= size(rb,2);
lnes(1,:)= zeros(1,len)+ xv(1);
lnes(2,:)= zeros(1,len)+ xv(2);
lnes(3:4,:)= TransformToGlobal([rb(1,:).*cos(rb(2,:)); rb(1,:).*sin(rb(2,:))], xv);
p= line_plot_conversion (lnes);

function data= initialise_store(x, xtrue)
% offline storage initialisation
data.i=1;
data.path= x;
data.true= xtrue;
data.state(1).x= x;

function data= store_data(data, x, xtrue)
% add current data to offline storage
CHUNK= 50000;
if data.i == size(data.path,2) % grow array in chunks to amortise reallocation
    data.path= [data.path zeros(3,CHUNK)];
    data.true= [data.true zeros(3,CHUNK)];
end
i= data.i + 1;
data.i= i;
data.path(:,i)= x(1:3);
data.true(:,i)= xtrue;
data.state(i).x= x;

