function [trajectoryState, mapState, lmLastSeenTable]= MCMC_build( trajectoryState, mapState, GVec, zVec, lmLastSeenTable )

format compact
path(path, '../')
configfile;


% Building a consistent map with the latest observations
% get the last controls that led to stateInd
Gseq = GVec(:,size(GVec,2));
stateInd = GVec(size(GVec,1),size(GVec,2));

% states
x= trajectoryState(:,size(trajectoryState,2));
for i=1:size(Gseq)-1
    x= predict_true(x, V,Gseq(i), WHEELBASE, DT_CONTROLS);
end
trajectoryState = [trajectoryState, x];

% map feature points
for i = 1:size(zVec,2)
    zV = zVec(:,i);
    if( zV(1) == stateInd ) % new remark
        if( lmLastSeenTable(1, zV(4)) == 0 ) % this feature is seen first => this goes to the spanning tree!
            f = zeros(2,1);
            f(1) = x(1) + zV(2)*cos(x(3) + zV(3));
            f(2) = x(2) + zV(2)*sin(x(3) + zV(3));
            mapState = [mapState, f];
        end
        lmLastSeenTable(1, zV(4)) = stateInd;
        lmLastSeenTable(2, zV(4)) = size(mapState,2);
    end
end

end