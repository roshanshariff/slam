%recursive function call. call this for all children
function [stateCoordinates, featureCoordinates] = getSLAMstates( parentIDInd, parentCoord,  igr )
    ig = igr.m_inferenceGraph;
    stateCoordinates   = zeros( 3, size( ig.m_stateLinks,2 ) );
    featureCoordinates = zeros( 3, size( ig.m_featureLinks,2 ) );
nodeRealizations = getSLAMnodeRealizations
    
    % set the first state to (0,0), and walk thru the spanning tree, calculating all state values
    rootIDInd = ig.m_spanningTreeRootIDInd;
    rootCoord = zeros(3,1);
%    stateCoordinates( :, rootIDInd ) = rootCoord;
%        if link(1) == 1  %dynamical model
%            if ig.m_edges(edgeInd, 1) == ig.m_stateIDs( parentIDInd )
%                childSC = parentCoord + realization; 
%            else
%                childSC = parentCoord - realization;                 
%            end
%            childCoord = childSC;
%        else             %observation model
%            if ig.m_edges(edgeInd, 1) == ig.m_stateIDs( parentIDInd )
%                childFC = parentCoord + realization; 
%            else
%                childFC = parentCoord - realization;                 
%            end            
%            childCoord = childFC;
%        end

    
    
    %recursive call for all children
    for i=1:(ig.m_spanningTreeChilds.at(parentIDInd).vsize())
        childIDInd  = ig.m_spanningTreeChilds.at(parentIDInd).at(i);
        edgeInd     = ig.m_spanningTreeParentEdgeInd( childIDInd );
        link        = ig.m_links(edgeInd, :);
        realization = igr.m_edgeRealizations(:, edgeInd);    
        
        childSC = [];
        childFC = [];
        if link(1) == 1  %dynamical model
            if ig.m_edges(edgeInd, 1) == ig.m_stateIDs( parentIDInd )
                %childSC = parentCoord + realization;
                theta      = parentCoord(3); 
                childSC    = zeros(3,1);
                childSC(1) = cos( theta )*realization(1) - sin( theta )*realization(2) + parentCoord(1);
                childSC(2) = sin( theta )*realization(1) + cos( theta )*realization(2) + parentCoord(2);
                childSC(3) = realization(3) + theta;
            else
                childSC = parentCoord - realization;                 
            end
            childCoord = childSC;
        else             %observation model
            if ig.m_edges(edgeInd, 1) == ig.m_stateIDs( parentIDInd )
                %childFC = parentCoord + realization; 
                theta      = parentCoord(3); 
                childFC    = zeros(3,1);
                childFC(1) = cos( theta )*realization(1) - sin( theta )*realization(2) + parentCoord(1);
                childFC(2) = sin( theta )*realization(1) + cos( theta )*realization(2) + parentCoord(2);
                childFC(3) = realization(3) + theta;
            else
                childFC = parentCoord - realization;                 
            end            
            childCoord = childFC;
        end        
        
        [sc,fc] = getSLAMstates( childIDInd, childCoord, igr );
        stateCoordinates = [stateCoordinates, childSC, sc];    
        featureCoordinates = [featureCoordinates, childFC, fc];    
    end
   
end

% we return coordinates according to the stateIDList
function nodeRealizations = getSLAMnodeRealizations( igr )
        
    % set the first state to (0,0), and walk thru the spanning tree, calculating all state values
    rootIDInd = ig.m_spanningTreeRootIDInd;
    rootCoord = zeros(3,1);
    nodeRealizations( :, rootIDInd ) = rootCoord;
    
    openStateInds = Vector();
    for j=1:ig.m_spanningTreeChilds.at(rootIDInd).vsize()
    	openStateInds.push_back( [ig.m_spanningTreeChilds.at(rootIDInd).at(j); rootIDInd] );
	end
%    for i=1:openStateInds.vsize()
    i=1;
    while i<=openStateInds.vsize()
        vv          = openStateInds.v(:,i);
        ind         = vv(1);
        parentIDInd = vv(2);
        
        for j=1:ig.m_spanningTreeChilds.at(ind).vsize()
            openStateInds.push_back( [ig.m_spanningTreeChilds.at(ind).at(j); ind] );
        end
        
        edgeInd     = ig.m_spanningTreeParentEdgeInd( ind );
        link        = ig.m_links(edgeInd,:);
        realization = igr.m_edgeRealizations(:, edgeInd); 
        parentCoord = nodeRealizations( :, parentIDInd );
        
        if link(1) == 1  %dynamical model
            if ig.m_edges(edgeInd, 1) == ig.m_stateIDs( parentIDInd )
                childSC = parentCoord + realization; 
            else
                childSC = parentCoord - realization;                 
            end
            childCoord = childSC;
        else             %observation model
            if ig.m_edges(edgeInd, 1) == ig.m_stateIDs( parentIDInd )
                childFC = parentCoord + realization; 
            else
                childFC = parentCoord - realization;                 
            end            
            childCoord = childFC;
        end

        nodeRealizations( :, ind ) = childCoord;
        i = i + 1;
    end       
end