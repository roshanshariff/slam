classdef InferenceGraphRealization < handle

   properties 
       m_inferenceGraph;               %the inference graph for which this class stores the realization
       
       m_edgeRealizations;             % the realization of inference graph edges
       
       m_edgeProb;                     % the probability (likelihood?) of graph edge realizations        
       m_logEdgeProb;                  % the log-probability (likelihood?) of graph edge realizations
       
%%TODO       m_absoluteCoords;               % stores the absolute coordinates
   end
   
   methods 
       function obj = InferenceGraphRealization( inferenceGraph )
           obj.m_inferenceGraph   = inferenceGraph;
           obj.m_edgeRealizations = [];
           obj.m_edgeProb         = [];
           obj.m_logEdgeProb      = [];
       end

       function obj = update( objIn, spanningTreeRealization )
           obj = objIn;
           % for all new edges add an edge realization
           for i = (size(obj.m_edgeRealizations,2)+1):size(obj.m_inferenceGraph.m_edges,1) 
                obj = addEdge( obj, spanningTreeRealization, i );
           end
       end
       
       function obj = addEdge( objIn, spanningTreeRealization, inferenceGraphEdgeIndex )  %with the edge index in the inference graph 
           obj = objIn;
           if obj.m_inferenceGraph.m_isSpanningTreeEdge( inferenceGraphEdgeIndex ) == 1
               edgeRealization = obj.m_inferenceGraph.sampleEdgeRealization( inferenceGraphEdgeIndex );
               spanningTreeRealization.addEdge( inferenceGraphEdgeIndex, edgeRealization );
           else
               stateID1    = obj.m_inferenceGraph.m_edges( inferenceGraphEdgeIndex, 1);
               stateIDInd1 = find(obj.m_inferenceGraph.m_stateIDs==stateID1);
               stateID2    = obj.m_inferenceGraph.m_edges( inferenceGraphEdgeIndex, 2);
               stateIDInd2 = find(obj.m_inferenceGraph.m_stateIDs==stateID2);
               edgeRealization = spanningTreeRealization.getRelativeCoordinateGeneral( stateIDInd1, stateIDInd2 );     
           end
           obj.m_edgeRealizations = [obj.m_edgeRealizations, edgeRealization];
           obj.m_edgeProb         = [obj.m_edgeProb, obj.m_inferenceGraph.getEdgeProbability( inferenceGraphEdgeIndex, edgeRealization )];
           obj.m_logEdgeProb      = [obj.m_logEdgeProb, obj.m_inferenceGraph.getEdgeLogProbability( inferenceGraphEdgeIndex, edgeRealization )];
       end
       
       
       function p = getEdgeProbabilities( obj, edgeInds )
           p =[];
           for i=1:size(edgeInds,2)
               p=[p; obj.m_edgeProb(edgeInds(i))];
           end
       end

       function p = getLogEdgeProbabilities( obj, edgeInds )
           p =[];
           for i=1:size(edgeInds,2)
               p=[p; obj.m_logEdgeProb(edgeInds(i))];
           end
       end

       function alpha = getAlpha( obj )
            alpha = 0;
            for i=1:size(obj.m_inferenceGraph.m_spanningTreeEdgeInds,2)
                alpha = alpha + 1/obj.m_edgeProb(obj.m_inferenceGraph.m_spanningTreeEdgeInds(i));
            end
       end
              
       function obj = setNewRealization( objIn, edgeIndex, edgeRealization, edgeProbability, logEdgeProbability )  %with the edge index in the inference graph
           obj = objIn;
           obj.m_edgeRealizations(:, edgeIndex) = edgeRealization;
           obj.m_edgeProb(edgeIndex)            = edgeProbability;
           obj.m_logEdgeProb(edgeIndex)         = logEdgeProbability;
       end
       
       function nodeRealizations = getSLAMnodeRealizations( obj )
            ig = obj.m_inferenceGraph;
            if size(ig.m_stateIDs,2) < 1
                nodeRealizations = zeros( 3, 1 );
                return
            end

            nodeRealizations = zeros( 3, size(ig.m_stateIDs,2) );

            % set the first state to (0,0), and walk thru the spanning tree, calculating all state values
            rootIDInd = ig.m_spanningTreeRootIDInd;
            rootCoord = zeros(3,1);
            nodeRealizations( :, rootIDInd ) = rootCoord;

%             openStateInds = Vector();
%             for j=1:ig.m_spanningTreeChilds.at(rootIDInd).vsize()
%                 openStateInds.push_back( [ig.m_spanningTreeChilds.at(rootIDInd).at(j); rootIDInd] );
%             end
%             i=1;
%             while i<=openStateInds.vsize()
%                 vv          = openStateInds.v(:,i);
%                 ind         = vv(1);
%    
%                 for j=1:ig.m_spanningTreeChilds.at(ind).vsize()
%                     openStateInds.push_back( [ig.m_spanningTreeChilds.at(ind).at(j); ind] );
%                 end
%                 i = i + 1;
%             end            
% 
%             
%             if ig.m_absCoordProcessOrder ~= openStateInds.v
%                 ig.m_absCoordProcessOrder
%                 openStateInds.v
%             end
            
            for i=1:size(ig.m_absCoordProcessOrder,2)
                vv          = ig.m_absCoordProcessOrder(:,i);
                ind         = vv(1);
                parentIDInd = vv(2);
                
                edgeInd     = ig.m_spanningTreeParentEdgeInd( ind );
                link        = ig.m_links(edgeInd,:);
                if edgeInd > size( obj.m_edgeRealizations, 2 )
                    edgeInd
                end
                realization = obj.m_edgeRealizations(:, edgeInd); 
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
            end       
           
       end
   end
end 
