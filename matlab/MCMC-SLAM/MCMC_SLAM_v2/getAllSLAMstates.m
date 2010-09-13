% we return coordinates according to the stateIDList
function [stateCoordinates, featureCoordinates, featureIdx] = getAllSLAMstates( igr )
    nodeRealizations = getSLAMnodeRealizations( igr );
    ig = igr.m_inferenceGraph;
    stateCoordinates   = [];
    featureCoordinates = [];
    featureIdx         = [];

    if size(nodeRealizations,2) < 2
        return;
    end
    
    for i = 1:size(nodeRealizations,2)
        if ig.m_stateIDs(i) < 0
            featureCoordinates = [featureCoordinates, nodeRealizations(:, i)];
            featureIdx         = [featureIdx, -ig.m_stateIDs(i)];
        else
            stateCoordinates   = [stateCoordinates, nodeRealizations(:, i)];
        end    
    end
end