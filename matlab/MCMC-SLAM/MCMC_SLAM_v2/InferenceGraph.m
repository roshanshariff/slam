classdef InferenceGraph < handle

   properties 
       m_stateIDs;                     % the state IDs
       m_edges;                        % the edges (pairs of stateIDs)
       
       m_links;                        % m_links[i,1] = index means this is a state link and m_stateLinks[index] has the data
       m_stateLinks;                   % state links
       m_featureLinks;                 % m_links[i] is the probability density of m_edges[i]
             
       m_spanningTreeEdgeInds;         % the edge indexes that belongs to the spanning tree of the Inference Graph       
       m_spanningTreeParent;           % m_spanningTreeParent(i) is the parent of m_stateIDs(i). (the parent of a spanning tree
                                       % vertex is the vertex of which it
                                       % first occured.
                                   
       m_spanningTreeRootIDInd;        %the root ID Ind of the spanning tree.
       m_spanningTreeChilds;           % m_spanningTreeChilds(i) is the child vertexes inds of m_stateIDs(i) 
       m_spanningTreeParentEdgeInd;    % m_spanningTreeParentEdgeInd(i) is the edge ind which connects m_stateIDs(i) to its parent. 
       
       m_changingEdgeInds;             % m_changingEdges(i) is as follows: (i must be a spannign tree edge)
                                       % m_spanningTreeEdgeInds(i) separates the spanning tree into two sets. 
                                       % m_changingEdgeInds(i,1) holds the edge indexes which has its end in two different sets
                                       % excluding the spanning-tree edge,
                                       % while m_changingEdgeInds(i,2)
                                       % holds the node that is changing
                                       % its absolute coordinate ( not the
                                       % same sub-tree as the root )
        
        m_isSpanningTreeEdge;
        
        m_absCoordProcessOrder;        %was openStateInd in getSLAMnodeRealizations 
   end
   
   methods 
       function obj = InferenceGraph()
           obj.m_stateIDs             = [];
           obj.m_edges                = [];
           obj.m_links                = [];
           obj.m_stateLinks           = [];
           obj.m_featureLinks         = [];
           obj.m_spanningTreeEdgeInds = [];
           obj.m_isSpanningTreeEdge   = [];
           obj.m_spanningTreeChilds   = Vector();
           obj.m_absCoordProcessOrder = [];
       end

       function obj = addEdge( objIn, stateID1, stateID2, link, type )
            obj = objIn;
            obj.m_edges            = [obj.m_edges;[stateID1,stateID2]];
            obj.m_changingEdgeInds = [obj.m_changingEdgeInds,Vector()];
   
            if (sum(type == 'dyn') == 3)
                obj.m_stateLinks = [obj.m_stateLinks, link];
                obj.m_links      = [obj.m_links; 1,size(obj.m_stateLinks,2)];
            else
                obj.m_featureLinks = [obj.m_featureLinks, link];
                obj.m_links      = [obj.m_links; 2,size(obj.m_featureLinks,2)];
            end
            
            %simple algorithm for spanning tree update: If stateID not
            %exists, add the new link to the spanning tree
            stateID1Index = find(obj.m_stateIDs==stateID1);
            stateID2Index = find(obj.m_stateIDs==stateID2);
            %the first edge
            addToSpanningTree = 0;
            if( size(stateID1Index,2) == 0 && size(stateID2Index,2) == 0 ) 
                obj.m_stateIDs    = [obj.m_stateIDs, stateID1];
                obj.m_stateIDs    = [obj.m_stateIDs, stateID2];
                parentIndex       = size(obj.m_stateIDs,2)-1;
                obj.m_spanningTreeRootIDInd = parentIndex;
                obj.m_spanningTreeChilds.push_back( Vector() ); % the child does not have children yet
                obj.m_spanningTreeParent = -1;
                obj.m_spanningTreeParentEdgeInd = -1
                addToSpanningTree = 2;
            elseif( size(stateID1Index,2) == 0 && size(stateID2Index,2) > 0 ) 
                obj.m_stateIDs    = [obj.m_stateIDs, stateID1];
                addToSpanningTree = 1;
                parentIndex = stateID2Index;
            elseif( size(stateID2Index,2) == 0 && size(stateID1Index,2) > 0 ) 
                obj.m_stateIDs    = [obj.m_stateIDs, stateID2];
                addToSpanningTree = 1;
                parentIndex = stateID1Index;
            end        
            if( addToSpanningTree > 0 )
                childIndex                                  = size(obj.m_stateIDs,2);
                obj.m_spanningTreeEdgeInds                  = [obj.m_spanningTreeEdgeInds, size( obj.m_edges, 1 )];
%                obj.m_changingEdgeInds                      = [obj.m_changingEdgeInds,Vector()];
%                obj.m_spanningTreeParent(childIndex)        = parentIndex;
%                obj.m_spanningTreeParentEdgeInd(childIndex) = size( obj.m_edges, 1 );
                obj.m_spanningTreeParent        = [obj.m_spanningTreeParent,parentIndex];
                obj.m_spanningTreeParentEdgeInd = [obj.m_spanningTreeParentEdgeInd, size( obj.m_edges, 1 )];
                
                obj.m_spanningTreeChilds.push_back( Vector() ); % the child does not have children yet
                obj.m_spanningTreeChilds.at(parentIndex).push_back( childIndex );
                
                obj.m_absCoordProcessOrder = [ obj.m_absCoordProcessOrder, [childIndex; parentIndex] ];
                
                obj.m_isSpanningTreeEdge = [obj.m_isSpanningTreeEdge,1];
            else
                obj.m_isSpanningTreeEdge = [obj.m_isSpanningTreeEdge,0];
                % get the path on the spanning tree between stateID1 and 
                % stateID2. this edge belongs to the changing index for all
                % edges of this path
                % The parents of the IDs should meet somewhere
                idx1 = stateID1Index;
                parentLine1 = [idx1];
                while( obj.m_spanningTreeParent(idx1) > 0 )
                    idx1 = obj.m_spanningTreeParent(idx1);
                    parentLine1 = [parentLine1, idx1];
                end
                
                idx2 = stateID2Index;
                parentLine2 = [idx2];
                while( obj.m_spanningTreeParent(idx2) > 0 )
                    idx2 = obj.m_spanningTreeParent(idx2);
                    parentLine2 = [parentLine2, idx2];
                end
                
                % the state where the parentLines meet are not in the path
                i1 = size(parentLine1,2);
                i2 = size(parentLine2,2);
                while( parentLine1(i1) == parentLine2(i2) )
                    i1=i1-1;
                    i2=i2-1;
                end
                
                %get the edges of these parentLines
                for i = 1:i1
                    edgeIndex = obj.m_spanningTreeParentEdgeInd( parentLine1(i) );
                    obj.m_changingEdgeInds(edgeIndex).push_back( [size(obj.m_edges,1); stateID1Index ] );
                end
                for i = 1:i2
                    edgeIndex = obj.m_spanningTreeParentEdgeInd( parentLine2(i) );
                    obj.m_changingEdgeInds(edgeIndex).push_back( [size(obj.m_edges,1); stateID2Index] );
                end
                
            end
       end
       
       %public methods needed by MCMC TODO: do we really need to copy this?
       function edgeNumber = getSpanningTreeEdgeNumber( obj )
           edgeNumber = size(obj.m_spanningTreeEdgeInds,2);
       end
       
       function edgeInd = getSpanningTreeEdgeInd( obj, index )
           edgeInd = obj.m_spanningTreeEdgeInds(index);
       end
       
       function p = getEdgeProbability( obj, edgeIndex, edgeRealization )
           if( obj.m_links(edgeIndex,1)==1 )
               p = obj.m_stateLinks( obj.m_links(edgeIndex,2) ).getLinkProbability( edgeRealization );
           else
               p = obj.m_featureLinks( obj.m_links(edgeIndex,2) ).getLinkProbability( edgeRealization );
           end
       end

       function p = getEdgeLogProbability( obj, edgeIndex, edgeRealization )
           if( obj.m_links(edgeIndex,1)==1 )
               p = obj.m_stateLinks( obj.m_links(edgeIndex,2) ).getLogLinkProbability( edgeRealization );
           else
               p = obj.m_featureLinks( obj.m_links(edgeIndex,2) ).getLogLinkProbability( edgeRealization );
           end
       end
       
       %get links that connect the two sides of the spanning tree separated
       %by edgeIndex. edgeIndex must be a spanning tree edge-index
       function changeEdgeInds = getChangingEdgeInds( obj, index )
            changeEdgeInds = obj.m_changingEdgeInds( index );
       end

       function edgeRealization = sampleEdgeRealization( obj, edgeIndex )
           if( obj.m_links(edgeIndex,1)==1 )
               edgeRealization = obj.m_stateLinks( obj.m_links(edgeIndex,2) ).sampleLink();
           else
               edgeRealization = obj.m_featureLinks( obj.m_links(edgeIndex,2) ).sampleLink();
           end
       end           
   end
end 
