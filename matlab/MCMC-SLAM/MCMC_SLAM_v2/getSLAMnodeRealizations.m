% we return coordinates according to the stateIDList
function nodeRealizations = getSLAMnodeRealizations( igr )
    ig = igr.m_inferenceGraph;
    if size(ig.m_stateIDs,2) < 1
        nodeRealizations = zeros( 3, 1 );
        return
    end

    nodeRealizations = zeros( 3, size(ig.m_stateIDs,2) );
        
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
        if edgeInd > size( igr.m_edgeRealizations, 2 )
            edgeInd
        end
        realization = igr.m_edgeRealizations(:, edgeInd); 
        parentCoord = nodeRealizations( :, parentIDInd );
        
        if link(1) == 1  %dynamical model
            if ig.m_edges(edgeInd, 1) == ig.m_stateIDs( parentIDInd )
                theta      = parentCoord(3); 
                childSC    = zeros(3,1);
                childSC(1) = cos( theta )*realization(1) - sin( theta )*realization(2) + parentCoord(1);
                childSC(2) = sin( theta )*realization(1) + cos( theta )*realization(2) + parentCoord(2);
                if ig.m_edges(edgeInd, 1) > 0 && ig.m_edges(edgeInd, 2) > 0
                    childSC(3) = realization(3) + theta;           
                end
            else
                childSC = TODO - parentCoord - realization;                 
            end
            childCoord = childSC;
        else             %observation model
            if ig.m_edges(edgeInd, 1) == ig.m_stateIDs( parentIDInd )
                theta      = parentCoord(3); 
                childFC    = zeros(3,1);
                childFC(1) = cos( theta )*realization(1) - sin( theta )*realization(2) + parentCoord(1);
                childFC(2) = sin( theta )*realization(1) + cos( theta )*realization(2) + parentCoord(2);
                if ig.m_edges(edgeInd, 1) > 0 && ig.m_edges(edgeInd, 2) > 0
                    childFC(3) = realization(3) + theta;
                end
            else
                childFC = TODO - parentCoord - realization;                 
            end            
            childCoord = childFC;
        end

        nodeRealizations( :, ind ) = childCoord;
        i = i + 1;
    end       
end