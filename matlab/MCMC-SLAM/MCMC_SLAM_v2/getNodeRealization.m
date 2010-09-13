function nodeRealization = getNodeRealization( inferenceGraphRealization, inferenceGraphEdgeIndex, edgeRealization, nodeRealizations,  at )
    ig = inferenceGraphRealization.m_inferenceGraph;

    %check if at is th right side of inferenceGraphEdgeIndex
    edge = ig.m_edges( inferenceGraphEdgeIndex, : );
    if ig.m_stateIDs( at ) ~= edge(2) 
        hiba TODO
    end
    parentIDInd = find( ig.m_stateIDs == edge(1) );
    link        = ig.m_links(inferenceGraphEdgeIndex,:);
    parentCoord = nodeRealizations( :, parentIDInd );
        
    if link(1) == 1  %dynamical model
    	if ig.m_edges(inferenceGraphEdgeIndex, 1) == ig.m_stateIDs( parentIDInd )
        	theta      = parentCoord(3); 
            childSC    = zeros(3,1);
            childSC(1) = cos( theta )*edgeRealization(1) - sin( theta )*edgeRealization(2) + parentCoord(1);
            childSC(2) = sin( theta )*edgeRealization(1) + cos( theta )*edgeRealization(2) + parentCoord(2);
            if ig.m_edges(inferenceGraphEdgeIndex, 1) > 0 && ig.m_edges(inferenceGraphEdgeIndex, 2) > 0
                childSC(3) = edgeRealization(3) + theta;           
            end
        else
            childSC = TODO - parentCoord - realization;                 
        end
        nodeRealization = childSC;
    else             %observation model
    	if ig.m_edges(inferenceGraphEdgeIndex, 1) == ig.m_stateIDs( parentIDInd )
        	theta      = parentCoord(3); 
            childFC    = zeros(3,1);
            childFC(1) = cos( theta )*edgeRealization(1) - sin( theta )*edgeRealization(2) + parentCoord(1);
            childFC(2) = sin( theta )*edgeRealization(1) + cos( theta )*edgeRealization(2) + parentCoord(2);
            if ig.m_edges(inferenceGraphEdgeIndex, 1) > 0 && ig.m_edges(inferenceGraphEdgeIndex, 2) > 0
            	childFC(3) = edgeRealization(3) + theta;
            end
        else
        	childFC = TODO - parentCoord - realization;                 
        end            
        nodeRealization = childFC;
    end
end