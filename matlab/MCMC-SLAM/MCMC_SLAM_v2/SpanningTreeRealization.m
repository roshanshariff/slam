classdef SpanningTreeRealization < handle
% the realization is stored in a tricky manner. For each node of the tree
% we store its absolute coordinate that is consistent to some of if
% parents... TODO: write it in a more understandable way
    
   properties        
       m_inferenceGraphRealization;
       m_nodeRealizations;             % the realization of inference graph edges
       m_subTreeTransformation;        % the transformation that should be applied for all nodes in the sub-tree below 
       m_hasTransformation;
   end
   
   methods 
       function obj = SpanningTreeRealization( inferenceGraphRealization )
           obj.m_inferenceGraphRealization = inferenceGraphRealization;

           obj.m_nodeRealizations      = obj.m_inferenceGraphRealization.getSLAMnodeRealizations();
%           obj.m_nodeRealizations      = getSLAMnodeRealizations( obj.m_inferenceGraphRealization );
           obj.m_subTreeTransformation = zeros( 3, size(obj.m_nodeRealizations,2) );
           obj.m_hasTransformation     = zeros( size(obj.m_nodeRealizations,2), 1 );
       end
       
       function obj = addEdge( obj, inferenceGraphEdgeIndex, edgeRealization )
           igr = obj.m_inferenceGraphRealization;           
           obj.m_nodeRealizations      = [obj.m_nodeRealizations, getNodeRealization( igr, inferenceGraphEdgeIndex, edgeRealization, obj.m_nodeRealizations, size(obj.m_nodeRealizations,2)+1 )];
           obj.m_subTreeTransformation = [obj.m_subTreeTransformation, zeros(3,1)];
           obj.m_hasTransformation     = [obj.m_hasTransformation;0];
       end
       
       function v = getRelativeCoordinateGeneral( obj, origoStateIDIndex, stateIDIndex )
            % get the parentLine from stateIDIndex to origoStateIDIndex
            idx = stateIDIndex;
            parentLine = idx;
            while( idx > 0 )
            	idx = obj.m_inferenceGraphRealization.m_inferenceGraph.m_spanningTreeParent(idx);
                parentLine = [parentLine, idx];
            end
       
            idx = origoStateIDIndex;
            origoStateParentLine = idx;
            while( idx > 0 )
            	idx = obj.m_inferenceGraphRealization.m_inferenceGraph.m_spanningTreeParent(idx);
                origoStateParentLine = [origoStateParentLine, idx];
            end
                            
            % the state where the parentLines meet are not in the path
            i1 = size(parentLine,2);
            i2 = size(origoStateParentLine,2);
            while( parentLine(i1) == origoStateParentLine(i2) )
            	i1=i1-1;
            	i2=i2-1;
            end
            
            %the last last parent of both nodes are parentLine(i1+1)
            commonParentIdIndex = parentLine(i1+1);            
            v1 = getRelativeCoordinate( obj, commonParentIdIndex, origoStateIDIndex );
            v2 = getRelativeCoordinate( obj, commonParentIdIndex, stateIDIndex );
            
            %the vector in the commonParentIdIndex coordinate system
            dv = v2-v1;
            %rotate with v1(3);
            v = zeros(3,1);
            v(1) = cos( -v1(3) )*dv(1) - sin( -v1(3) )*dv(2); 
            v(2) = sin( -v1(3) )*dv(1) + cos( -v1(3) )*dv(2); 
            if obj.m_inferenceGraphRealization.m_inferenceGraph.m_stateIDs(stateIDIndex) > 0 && obj.m_inferenceGraphRealization.m_inferenceGraph.m_stateIDs(origoStateIDIndex) > 0 %only on state edges
            	v(3) = vv(3);                     
            end
       end
       
       function v =  getRelativeCoordinate( obj, origoStateIDIndex, stateIDIndex )
%             % get the parentLine from stateIDIndex to origoStateIDIndex
%             idx = stateIDIndex;
%             parentLine = idx;
%             while( idx ~= origoStateIDIndex )
%             	idx = obj.m_inferenceGraphRealization.m_inferenceGraph.m_spanningTreeParent(idx);
%                 parentLine = [parentLine, idx];
%             end
            
            %calculate the relative coordinate
            stateID      = obj.m_inferenceGraphRealization.m_inferenceGraph.m_stateIDs( stateIDIndex );  
            origoStateID = obj.m_inferenceGraphRealization.m_inferenceGraph.m_stateIDs( origoStateIDIndex );
            coord        = obj.m_nodeRealizations( :, stateIDIndex );
%             for i=1:size(parentLine,2)
%                 if( obj.m_hasTransformation( parentLine(i) ) )
% 
%                    % the origo of the transformation
%                    ox = obj.m_nodeRealizations( 1, parentLine(i) );
%                    oy = obj.m_nodeRealizations( 2, parentLine(i) );
%                    %the transformation
%                    dx     = obj.m_subTreeTransformation( 1, parentLine(i) );
%                    dy     = obj.m_subTreeTransformation( 2, parentLine(i) );
%                    dtheta = obj.m_subTreeTransformation( 3, parentLine(i) );
% 
%                    c        = coord;
%                    coord(1) = cos( dtheta )*(c(1)-ox) - sin( dtheta )*(c(2)-oy) + ox + dx;
%                    coord(2) = sin( dtheta )*(c(1)-ox) + cos( dtheta )*(c(2)-oy) + oy + dy;
%                    if stateID > 0 && origoStateID > 0
%                        coord(3) = coord(3) + dtheta;
%                    end
%                 end
%             end           
            vv = coord - obj.m_nodeRealizations( :, origoStateIDIndex );
            if stateID < 0 || origoStateID < 0
                vv(3) = 0;
            end
            
            % the x-axis should face the same direction as the origo's
            % facing
            theta = obj.m_nodeRealizations( 3, origoStateIDIndex );
            v = vv; %just to get its size right :-)
            v(1) = cos( -theta )*vv(1) - sin( -theta )*vv(2);
            v(2) = sin( -theta )*vv(1) + cos( -theta )*vv(2);
       end
       
       function obj = setTransformation( objIn, origoStateInd, transformation )
            obj = objIn;
            obj.m_subTreeTransformation( :, origoStateInd ) = obj.m_subTreeTransformation( :, origoStateInd ) + transformation;
            obj.m_hasTransformation( origoStateInd )     = 1;
       end
       
       function edgeLogLikes = getEdgeLogLikes( obj )
            edges = obj.m_inferenceGraphRealization.m_inferenceGraph.m_edges;
            edgeLogLikes = [];
            for i = 1:size(edges, 1)
                edgeInd1 = find(obj.m_inferenceGraphRealization.m_inferenceGraph.m_stateIDs==edges(i,1));
                edgeInd2 = find(obj.m_inferenceGraphRealization.m_inferenceGraph.m_stateIDs==edges(i,2));
                edgeRealization = obj.m_nodeRealizations( :, edgeInd2 ) - obj.m_nodeRealizations( :, edgeInd1 );
                edgeLogLikes = [edgeLogLikes, obj.m_inferenceGraphRealization.m_inferenceGraph.getEdgeLogProbability( i, edgeRealization )];
            end
       end
   end
end