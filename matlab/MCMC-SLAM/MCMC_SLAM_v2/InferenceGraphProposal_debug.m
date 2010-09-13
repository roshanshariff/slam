classdef InferenceGraphProposal < handle

   properties 
       m_inferenceGraphRealization;                       % the realization the proposal starts from

       m_spanningTreeChangingEdgeInd;                     % which spanning tree edge is selected to change
       m_proposedChangeEdgeInds;                          % the change edge indices proposed to change
       m_proposedEdgeRealizations;                        % the  proposed edge realizations
       m_proposedEdgeProb;                                % their probabilities
       m_proposedLogEdgeProb;                             % their log probabilities
   end
   
   methods 
       function obj = InferenceGraphProposal( inferenceGraphRealization )
           obj.m_inferenceGraphRealization = inferenceGraphRealization;
       end

       function [obj,sumLogChangeProbDelta, transformationOrigoStateIDidx, transformation] = propose( objIn, spanningTreeRealization )  %adding a state means adding a state-state link
           obj = objIn;
           inferenceGraph = obj.m_inferenceGraphRealization.m_inferenceGraph;
           
           spanningTreeEdgeNo = inferenceGraph.getSpanningTreeEdgeNumber();
           dens  = [];
           for i = 1:spanningTreeEdgeNo
               edgeIndex = inferenceGraph.getSpanningTreeEdgeInd( i );
               p         = obj.m_inferenceGraphRealization.m_edgeProb(edgeIndex);
               dens  = [dens, 1/p];
           end
           alpha = sum(dens);
           %drawing spanning tree edge index to change drawing new
           %realization for the changing edge
           spanningTreeChangingInd         = logSampling( dens );
           spanningTreeChangingEdgeInd     = inferenceGraph.getSpanningTreeEdgeInd( spanningTreeChangingInd );
           newSpanningTreeEdgeRealization  = inferenceGraph.sampleEdgeRealization( spanningTreeChangingEdgeInd );
       
           obj.m_spanningTreeChangingEdgeInd = spanningTreeChangingEdgeInd;
           obj.m_proposedChangeEdgeInds      = spanningTreeChangingEdgeInd;
           obj.m_proposedEdgeRealizations    = newSpanningTreeEdgeRealization;
           obj.m_proposedEdgeProb            = inferenceGraph.getEdgeProbability( spanningTreeChangingEdgeInd, newSpanningTreeEdgeRealization );
           obj.m_proposedLogEdgeProb         = inferenceGraph.getEdgeLogProbability( spanningTreeChangingEdgeInd, newSpanningTreeEdgeRealization );

           %get transformation of the spanning tree edge
           ig = obj.m_inferenceGraphRealization.m_inferenceGraph;
           edge = ig.m_edges( spanningTreeChangingEdgeInd, : );
           if ig.m_spanningTreeParent(edge(1)) == edge(2)               
                transformationOrigoStateIDidx = find(ig.m_stateIDs==edge(1)); %in our spanning tree always the child transforms, as it is not in the same sub-tree as the root
           else
                transformationOrigoStateIDidx = find(ig.m_stateIDs==edge(2));
           end               
           transformation = getTransformation( obj.m_inferenceGraphRealization.m_edgeRealizations( :,spanningTreeChangingEdgeInd ), newSpanningTreeEdgeRealization );
           
           vxt1t  = obj.m_inferenceGraphRealization.m_edgeRealizations( :,spanningTreeChangingEdgeInd );
           vxt1tp = newSpanningTreeEdgeRealization;
           
           % the transformation is in x_t-1's coordiate system change it to
           % x_t's coordinate system
           R = getRotationMatrix( -obj.m_inferenceGraphRealization.m_edgeRealizations( 3,spanningTreeChangingEdgeInd ) );
           t = transformation;
           t(1:2) = R*transformation(1:2);
                      
           %transforming other changing edges
           sumLogChangeProbDelta = 0;
           changeEdgeInds = inferenceGraph.getChangingEdgeInds( spanningTreeChangingEdgeInd );
            
           %debugging!!!
           nodeRealizations    = getSLAMnodeRealizations( obj.m_inferenceGraphRealization );
           rotationOrigoCoord  = nodeRealizations(:, transformationOrigoStateIDidx );  
           
           oldSpanningTreeEdgeRealization = obj.m_inferenceGraphRealization.m_edgeRealizations(:, spanningTreeChangingEdgeInd);
           obj.m_inferenceGraphRealization.m_edgeRealizations(:, spanningTreeChangingEdgeInd) = newSpanningTreeEdgeRealization;           
           newNodeRealizations = getSLAMnodeRealizations( obj.m_inferenceGraphRealization );
           obj.m_inferenceGraphRealization.m_edgeRealizations(:, spanningTreeChangingEdgeInd) = oldSpanningTreeEdgeRealization;           
           for i=1:size(changeEdgeInds.v,2)
               % we have a rotation origo so we need the absolute
               % coordinates of the edge realizations
               % get the target state ID
               changeStateIDInd = changeEdgeInds.v(2,i); 
               % get the relative coordinate to transformation origo
               relCoord = spanningTreeRealization.getRelativeCoordinate( transformationOrigoStateIDidx, changeStateIDInd );
               
               vxtn = relCoord;
               vxnF = obj.m_inferenceGraphRealization.m_edgeRealizations( :, changeEdgeInds.v(1,i) );%OK
               
               vxt1n  = getTwoDistRelPos( vxt1t, vxtn );%OK
               vxt1np = getTwoDistRelPos( vxt1tp, vxtn );%OK
               R = getRotationMatrix(-vxt1np(3));
               vxnnp  = [ R*(vxt1n(1:2)-vxt1np(1:2)); -vxt1n(3)+vxt1np(3)];
               vxnpF = getTwoDistRelPos( vxnnp, vxnF );
               
               %get the transformation of relCoord in x_t's coordinate
               %system
               R = getRotationMatrix( transformation(3) );
               u = relCoord;
               u(1:2) = R*u(1:2);

               %get the transformation of the old edge realization in x_t's
               %coordinate system
               oldEdgeRealization = obj.m_inferenceGraphRealization.m_edgeRealizations( :, changeEdgeInds.v(1,i) );
               w = oldEdgeRealization;
               R = getRotationMatrix( -relCoord(3) );
               w(1:2) = R*w(1:2);
        
               v = w + relCoord - u - t; 
               newEdgeRealization11 = v;
               R = getRotationMatrix( -relCoord(3) );
               newEdgeRealization11(1:2) = R*v(1:2);

               
               % transform it with the transformation
               newRelCoord = zeros(3,1);
               newRelCoord(1) = cos( transformation(3) )*relCoord(1) - sin( transformation(3) )*relCoord(2) + transformation(1) ;
               newRelCoord(2) = sin( transformation(3) )*relCoord(1) + cos( transformation(3) )*relCoord(2) + transformation(2);
               newRelCoord(3) = relCoord(3) + transformation(3);
                             
               % transform the edge realization with the coordinate
               % transformation 
               v = oldEdgeRealization - ( newRelCoord - relCoord );
               newEdgeRealization1 = zeros(3,1);
               newEdgeRealization1(1) = cos( -transformation(3) )*v(1) - sin( -transformation(3) )*v(2); 
               newEdgeRealization1(2) = sin( -transformation(3) )*v(1) + cos( -transformation(3) )*v(2); 
                              
               
               % Let's do a way around!
               s1StateID = inferenceGraph.m_edges( changeEdgeInds.v(1,i), 2); 
               s1StateInd = find(ig.m_stateIDs==s1StateID);
               chFirst = 1; % the edge(1) is the changing index
               if s1StateInd == changeStateIDInd %ugly hack
                   chFirst = 0;
                   s1StateID = inferenceGraph.m_edges( changeEdgeInds.v(1,i), 1); 
                   s1StateInd = find(ig.m_stateIDs==s1StateID);
               end
               s1  = nodeRealizations(:, s1StateInd ); 
               s2  = nodeRealizations(:, changeStateIDInd ); 
               Ts2 = newNodeRealizations(:, changeStateIDInd );  
               
               TTs2 = Ts2;
               vuvu = s2 - rotationOrigoCoord;
               TTs2(1) = cos( -transformation(3) )*vuvu(1) - sin( -transformation(3) )*vuvu(2); 
               TTs2(2) = sin( -transformation(3) )*vuvu(1) + cos( -transformation(3) )*vuvu(2); 
               TTs2 = TTs2 + rotationOrigoCoord + transformation;                
               
               newEdgeRealization = zeros(3,1);
               if chFirst == 1 
                   % edge origo is Ts2
                    v = s1 - Ts2;
                    newEdgeRealization(1) = cos( -Ts2(3) )*v(1) - sin( -Ts2(3) )*v(2); 
                    newEdgeRealization(2) = sin( -Ts2(3) )*v(1) + cos( -Ts2(3) )*v(2); 
                    if ig.m_stateIDs(changeStateIDInd) > 0 && s1StateID > 0 %only on state edges
                        newEdgeRealization(3) = v(3);                     
                    end
               else
                   % edge origo is s1
                    v = Ts2 - s1;
                    newEdgeRealization(1) = cos( -s1(3) )*v(1) - sin( -s1(3) )*v(2); 
                    newEdgeRealization(2) = sin( -s1(3) )*v(1) + cos( -s1(3) )*v(2); 
                    if ig.m_stateIDs(changeStateIDInd) > 0 && s1StateID > 0 %only on state edges
                        newEdgeRealization(3) = v(3);                     
                    end
               end                   

               err = sum( abs( newEdgeRealization - newEdgeRealization1 ) );
               if err > 0.0001
                   newEdgeRealization
                   newEdgeRealization1
                   err
               end
%               newEdgeRealization = newEdgeRealization1;
               
               
               
               
               newEdgeProb        = inferenceGraph.getEdgeProbability( changeEdgeInds.v(1,i), newEdgeRealization );
               newLogEdgeProb     = inferenceGraph.getEdgeLogProbability( changeEdgeInds.v(1,i), newEdgeRealization );
                
               obj.m_proposedChangeEdgeInds   = [obj.m_proposedChangeEdgeInds, changeEdgeInds.v(1,i)];
               obj.m_proposedEdgeRealizations = [obj.m_proposedEdgeRealizations, newEdgeRealization];
               obj.m_proposedEdgeProb         = [obj.m_proposedEdgeProb, newEdgeProb]; 
               obj.m_proposedLogEdgeProb      = [obj.m_proposedLogEdgeProb, newLogEdgeProb]; 

               oldLogEdgeProb        = obj.m_inferenceGraphRealization.m_logEdgeProb( changeEdgeInds.v(1,i) );
               sumLogChangeProbDelta = sumLogChangeProbDelta + newLogEdgeProb - oldLogEdgeProb;
           end                  
       end

       function alphaDiff = getAlphaDiff( obj )
            alphaDiff = 1/obj.m_proposedEdgeProb(1) - 1/obj.m_inferenceGraphRealization.m_edgeProb(obj.m_spanningTreeChangingEdgeInd);
       end
       
       function coreChangeLikeRate = getCoreChangeLikeRate( obj )
            coreChangeLikeRate = obj.m_proposedEdgeProb(1)/obj.m_inferenceGraphRealization.m_edgeProb(obj.m_spanningTreeChangingEdgeInd);
       end
       
       
       function obj = accept( objIn )  %adding a state means adding a state-state link
           obj = objIn;
           for i=1:size(obj.m_proposedChangeEdgeInds,2)
                edgeIndex = obj.m_proposedChangeEdgeInds(i);
                obj.m_inferenceGraphRealization.setNewRealization( edgeIndex, obj.m_proposedEdgeRealizations(:, i), obj.m_proposedEdgeProb(i), obj.m_proposedLogEdgeProb(i) );
           end
       end
   end
end 
