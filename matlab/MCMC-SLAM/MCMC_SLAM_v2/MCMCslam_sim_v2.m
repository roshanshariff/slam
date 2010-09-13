function [lmErr, stateErr, time]= MCMCslam_sim_v2(lm, wp, MCMCStepNo, maxIterNo)
%function data= MCMCslam_sim_v2(lm, wp)
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
sumLogLike = [];
sumLogLike2 = [];
pcount=0;
lmLastSeenTable= zeros(2,size(lm,2)); % 0 if the landmark has not been seen, and holds the last state index where it was seen last 
xfp = [];

if SWITCH_SEED_RANDOM ~= 0, rand('state',SWITCH_SEED_RANDOM), randn('state',SWITCH_SEED_RANDOM), end

Qe= Q; Re= R;
if SWITCH_INFLATE_NOISE==1, Qe= 2*Q; Re= 2*R; end

%if SWITCH_PROFILE, profile on -detail builtin, end
dynamicalModel = DynamicalModel( V, Q, DT_CONTROLS, WHEELBASE );
observationModel = ObservationModel( R );
inferenceGraph = InferenceGraph();
mcmcAlg        = MCMConInferenceGraph(inferenceGraph);
%aviobj = avifile('c:/Works/MatlabProjects/MCMC_SLAM_v2/figures/video/video8.avi', 'compression', 'Cinepak');
%aviobj.quality = 100;

truePath = [];
lmErr = 0;
lmErrHistory = [];
stateErr = 0;
stateErrHistory = [];

% Main loop 
iterNo = 0;
logLikelihoodHistory = [];
errorHistory = [];
tic;
while iwp ~= 0 && iterNo < maxIterNo
    iterNo = iterNo + 1;
    
    % Compute true data
    [G,iwp]= compute_steering(xtrue, wp, iwp, AT_WAYPOINT, G, RATEG, MAXG, dt);
    if iwp==0 & NUMBER_LOOPS > 1, iwp=1; NUMBER_LOOPS= NUMBER_LOOPS-1; end % path loopfs repeat
    % Add process noise
    [Vn,Gn]= add_control_noise(V,G,Q, SWITCH_CONTROL_NOISE);
    xtrue= predict_true(xtrue, Vn,Gn, WHEELBASE,dt);
    G_ind = [G_ind, G];
    truePath = [truePath, xtrue];    
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
            

        [stateCoordinates, featureCoordinates, featureIdx] = getAllSLAMstates( mcmcAlg.m_inferenceGraphRealization );
        seenLM = zeros(2,size(featureCoordinates,2));
        for i=1:size(featureCoordinates,2)
            seenLM(:,i) = lm(:,featureIdx(i));
        end
        dTheta = getBestRotation( featureCoordinates, seenLM );
        RR = getRotationMatrix( dTheta );
        featureCoordinatesR = featureCoordinates;
        for i=1:size(featureCoordinates,2)
            featureCoordinatesR(1:2,i) = RR*featureCoordinates(1:2,i);
        end
        stateCoordinatesR = stateCoordinates;
        for i=1:size(stateCoordinates,2)
            stateCoordinatesR(1:2,i) = RR*stateCoordinates(1:2,i);
        end
%         if mod(stateIndex,30) == 2 
%             figure(111)
%             plot(lm(1,:),lm(2,:),'k*')
%             axis([-125 110 -100 100])
%             xlim([-125 110])
%             ylim([-100 100])
%             hold on, %axis equal
%             plot(wp(1,:),wp(2,:), wp(1,:),wp(2,:),'b', 'LineWidth', 2)
%             plot(featureCoordinatesR(1,:),featureCoordinatesR(2,:),'ro','LineWidth',2,'markersize', 6, 'erasemode','background'); % estimated features 
%             plot(stateCoordinatesR(1,:),stateCoordinatesR(2,:),stateCoordinatesR(1,:),stateCoordinatesR(2,:),'k','LineWidth',5,'erasemode','background'); % vehicle path estimate
%             plot(truePath(1,:),truePath(2,:),truePath(1,:),truePath(2,:),'g','LineWidth',2,'erasemode','background'); % vehicle path estimate
%         end
        
        lmErr = 0;
        for i=1:size(featureCoordinatesR,2)
            e = featureCoordinatesR(1:2,i) - seenLM(1:2,i);
            lmErr = lmErr + e'*e;
        end
        lmErr = lmErr/size(featureCoordinatesR,2);
%        lmErrHistory = [lmErrHistory, lmErr];
%        figure(112)
%        plot( lmErrHistory );
        
        stateErr = 0;
        for i=1:size(stateCoordinatesR,2)
            e = stateCoordinatesR(1:2,i) - truePath(1:2,i);
            stateErr = stateErr + e'*e;
        end
        stateErr = stateErr/size(stateCoordinatesR,2);
%        stateErrHistory = [stateErrHistory, stateErr];
%        figure(113)
%        plot( stateErrHistory );
        
        stateIndex = stateIndex + 1;

        if size(stateCoordinates,2)> 0
            data=store_data(data, stateCoordinates(:,size(stateCoordinates,2)), xtrue);
        end
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
        
    stateStateLink = StateStateLink( G, dynamicalModel );
    inferenceGraph.addEdge( stateIndex-1, stateIndex, stateStateLink, 'dyn' );
    for i = 1:size(zVec,2)
        featureStateLink = FeatureStateLink( zVec(2:3,i)', observationModel );
        inferenceGraph.addEdge( stateIndex, -zVec(4,i), featureStateLink, 'obs' );    
    end
    mcmcAlg.m_inferenceGraphRealization.update(mcmcAlg.m_spanningTreeRealization);
    mcmcAlg = mcmcAlg.step( MCMCStepNo );
    edgeLogLikes = mcmcAlg.m_spanningTreeRealization.getEdgeLogLikes();
    sumLogLike = [sumLogLike, sum( edgeLogLikes )];
%    figure(11);
%    plot( sumLogLike );
%    edgeLogLikes2 = mcmcAlg.m_inferenceGraphRealization.m_logEdgeProb;
%    sumLogLike2 = [sumLogLike2, sum( edgeLogLikes2 )];
%    plot( sumLogLike2, 'r' );

%    figure(12);
%    plot( edgeLogLikes );
%    plot( edgeLogLikes2, 'r' );
    
 %   F = getframe(figure(1));
 %   aviobj = addframe(aviobj,F);
    
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
    do_plot( h, xtrue, plines, veh );
end
lmErr
stateErr
time = toc;
%aviobj = close(aviobj);
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

