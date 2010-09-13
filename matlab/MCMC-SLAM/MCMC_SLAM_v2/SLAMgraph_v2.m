classdef SLAMgraph_v2 < handle

   properties 
       m_states;                       % the state coordinate realizations (including facing \theta)
       m_featureCoordinates;           % the feature coordinate ralizations
       
       m_stateStateLinkList;           % the list of state-state links
       m_featureStateLinkList;         % the list of feature-state links
       
       m_spanningTreeLinkIDList;       
       
       m_dynamicalModel;
       m_observationModel;
       
       m_stateNumber;
       
       m_linkIDmap;                     %maps the linkIDs to state or feature indexes
       m_linkProb;
       m_logLinkProb;
       
       m_featureIndexSeen;              % where
       m_featureIndexSeenList;          % m_featureIndexSeenList(fID): the list of links that were connected to feature (fID)
       
       m_connectedLinks;
       
       m_vertexMovedTime;               % in which step did  vertex i moved: m_vertexMovedTime[i] 
       m_edgeVertexInd;                 % m_edgeVertexInd[linkID] is a list of vertex indexes (2long list) of linkID
       m_vertexLinkIDs;                 % the list of linkIDs that are connected to vertex index i
       
       m_linkNo;
       m_vertexNo;
       m_stepNo;
       m_prevStateVertexIndex;             %the vertexIndex of the previous state...
       m_featureIndexMap;                  %maps the featureIndexes to graph vertex indexes
       m_stateIndexMap;                    %maps the state indexes to  graph vertex indexes
   end
   
   methods 
       function obj = SLAMgraph_v2( dynamicalModel, observationModel )
           obj.m_states = zeros(3,1);
           obj.m_stateNumber = 1;

           obj.m_featureCoordinates = [];
           
           obj.m_dynamicalModel   = dynamicalModel;
           obj.m_observationModel = observationModel;
           
           obj.m_featureIndexSeen = zeros(100,2);  % max number of features is 100
           
           obj.m_featureIndexSeenList = [];
           for i=1:100
               obj.m_featureIndexSeenList = [obj.m_featureIndexSeenList, Vector()];
           end
           
           obj.m_linkProb               = [];
           obj.m_logLinkProb            = [];
           obj.m_linkIDmap              = [];
           obj.m_spanningTreeLinkIDList = [];
           
           obj.m_connectedLinks = [];
           obj.m_vertexNo = 2;
           obj.m_stepNo = 2;
           obj.m_linkNo = 1;
           obj.m_prevStateVertexIndex = 1;
           obj.m_vertexMovedTime = [1];
           vv = Vector();
           obj.m_vertexLinkIDs = [vv];
           obj.m_stateIndexMap( 1 ) = 1;
       end

       function obj = addState( objIn, Control )  %adding a state means adding a state-state link
           obj = objIn;
           newStateLink         = StateStateLink( Control );
           obj.m_stateStateLinkList = [obj.m_stateStateLinkList, newStateLink];

           lastState = obj.m_states(:,obj.m_stateNumber);
           newState  = newStateLink.sampleLink( lastState, obj.m_dynamicalModel );
           obj.m_states  = [obj.m_states, newState];           
           
           lP = newStateLink.getLinkProbability( lastState, newState, obj.m_dynamicalModel );
           obj.m_linkProb  = [obj.m_linkProb, lP];
           logLP = newStateLink.getLogLinkProbability( lastState, newState, obj.m_dynamicalModel );
           obj.m_logLinkProb  = [obj.m_logLinkProb, logLP];
           
           obj.m_linkIDmap  = [obj.m_linkIDmap, [0; obj.m_stateNumber; obj.m_stateNumber+1; size(obj.m_stateStateLinkList,2)]];
           obj.m_stateNumber = obj.m_stateNumber + 1;
           
           obj.m_spanningTreeLinkIDList = [obj.m_spanningTreeLinkIDList, size(obj.m_linkIDmap,2)];
           v = Vector();
 %          v.push_back( size(obj.m_linkIDmap,2) );
           obj.m_connectedLinks = [obj.m_connectedLinks, v ];

           vv = Vector();
           vv.push_back( obj.m_linkNo );
           obj.m_vertexLinkIDs = [obj.m_vertexLinkIDs, vv];
           obj.m_edgeVertexInd = [obj.m_edgeVertexInd, [ obj.m_prevStateVertexIndex; obj.m_vertexNo]];
           obj.m_vertexMovedTime = [obj.m_vertexMovedTime, obj.m_stepNo]; 
           %there is a new link to the previous state vertex
           obj.m_vertexLinkIDs(obj.m_prevStateVertexIndex).push_back( obj.m_linkNo );
           obj.m_prevStateVertexIndex = obj.m_vertexNo;
           obj.m_stateIndexMap( obj.m_stateNumber ) = obj.m_vertexNo;

           
           obj.m_stepNo   = obj.m_stepNo + 1;
           obj.m_vertexNo = obj.m_vertexNo + 1;
           obj.m_linkNo   = obj.m_linkNo +1;
       end
       
       function obj = addObservedState( objIn, Control, zVec )  %adding a state means adding a state-state link
           obj = objIn;
           obj.addState( Control );
           lastState = obj.m_states(:,obj.m_stateNumber);
       
           %for all feature-stateLinks (from this state)
           for i = 1:size(zVec,2)
               zV = zVec(:,i);
               obsZ = [zV(2), zV(3)];
               newFeatureStateLink  = FeatureStateLink( obsZ );
               obj.m_featureStateLinkList = [obj.m_featureStateLinkList, newFeatureStateLink];
               obj.m_linkIDmap  = [obj.m_linkIDmap, [1; obj.m_stateNumber; zV(4); size(obj.m_featureStateLinkList,2)]];
               
               if( obj.m_featureIndexSeen(zV(4),1) == 0 ) % this feature is seen first => this goes to the spanning tree!
                   f = newFeatureStateLink.sampleLink( lastState, obj.m_observationModel );
                   obj.m_featureCoordinates = [obj.m_featureCoordinates, f];
                   obj.m_featureIndexSeen(zV(4),:) = [size(obj.m_linkIDmap,2), size(obj.m_featureCoordinates,2)];

                   obj.m_spanningTreeLinkIDList = [obj.m_spanningTreeLinkIDList, size(obj.m_linkIDmap,2)];           
                   v = Vector();
%                   v.push_back( size(obj.m_linkIDmap,2) );
                   obj.m_connectedLinks = [obj.m_connectedLinks, v ];

                   obj.m_featureIndexMap(zV(4)) = obj.m_vertexNo;
                   vv = Vector();
                   vv.push_back( obj.m_linkNo );
                   obj.m_vertexLinkIDs = [obj.m_vertexLinkIDs, vv];
                   obj.m_edgeVertexInd = [obj.m_edgeVertexInd, [ obj.m_prevStateVertexIndex; obj.m_vertexNo]];
                   obj.m_vertexMovedTime = [obj.m_vertexMovedTime, obj.m_stepNo]; 
                   obj.m_vertexLinkIDs(obj.m_prevStateVertexIndex).push_back( obj.m_linkNo );
                   
                   obj.m_stepNo   = obj.m_stepNo + 1;
                   obj.m_vertexNo = obj.m_vertexNo + 1;                   
               else
                   % UPDATE CONNECTED LINKS
                   % for the state-feature link, where this feature was
                   % seen first
                   firstAppearanceID = obj.m_featureIndexSeen(zV(4),1);
                   obj.m_connectedLinks(firstAppearanceID).push_back( size(obj.m_linkIDmap,2) );

                   % for all state-state links that lead to the last state
                   % since this feature has been seen first
                   for k = firstAppearanceID:size(obj.m_linkIDmap,2)
                       if( obj.m_linkIDmap(1,k) == 0 )
                           obj.m_connectedLinks(k).push_back( size(obj.m_linkIDmap,2) );
                       end
                   end
                   
                   obj.m_connectedLinks = [obj.m_connectedLinks, Vector() ];
                   
                   featureIndex = obj.m_featureIndexMap(zV(4));
                   obj.m_edgeVertexInd = [obj.m_edgeVertexInd, [obj.m_prevStateVertexIndex; featureIndex]];
                   obj.m_vertexLinkIDs(obj.m_prevStateVertexIndex).push_back( obj.m_linkNo );                   
                   obj.m_vertexLinkIDs(featureIndex).push_back( obj.m_linkNo );                   
               end
               obj.m_featureIndexSeenList(zV(4)).push_back( size(obj.m_linkIDmap,2) );
               featureCoord = obj.m_featureCoordinates( :, obj.m_featureIndexSeen(zV(4),2) );
               lP = newFeatureStateLink.getLinkProbability( lastState, featureCoord, obj.m_observationModel );
               obj.m_linkProb   = [obj.m_linkProb, lP];
               logLP = newFeatureStateLink.getLogLinkProbability( lastState, featureCoord, obj.m_observationModel );
               obj.m_logLinkProb   = [obj.m_logLinkProb, logLP];

               obj.m_linkNo   = obj.m_linkNo +1;
           end
       end

       function linkIDs = getAllLinkIDs( obj )
           linkIDs = 1:(obj.m_linkNo-1);
       end 
       
       
       %public methods needed by MCMC
       function linkIDs = getSpanningTreeLinkIDs( obj )
           linkIDs = obj.m_spanningTreeLinkIDList;
       end
       
       function movingVertex = getMovingVertexInd( obj, linkID )
           % which vertex moves
           v1 = obj.m_edgeVertexInd(1, linkID); 
           v2 = obj.m_edgeVertexInd(2, linkID); 
           if( obj.m_vertexMovedTime(v1) < obj.m_vertexMovedTime(v2) ) 
               movingVertex = v1;
           else
               movingVertex = v2;               
           end
       end
       
       function inLinkIDs = getInLinkIDs( obj, movingVertex )           
           inLinkIDs = [];
           for i = 1:obj.m_vertexLinkIDs(movingVertex).size()
               lID = obj.m_vertexLinkIDs(movingVertex).at(i);
               if( movingVertex == obj.getMovingVertexInd( lID ) )
                   inLinkIDs = [inLinkIDs, lID];
               end
           end
       end

       function outLinkIDs = getOutLinkIDs( obj, movingVertex )
           outLinkIDs = [];
           for i = 1:obj.m_vertexLinkIDs(movingVertex).size()
               lID = obj.m_vertexLinkIDs(movingVertex).at(i);
               if( movingVertex ~= obj.getMovingVertexInd( lID ) )
                   outLinkIDs = [outLinkIDs, lID];
               end
           end
       end       
       
       function p = getLinkProbabilities( obj, linkIDs )
           p =[];
           for i=1:size(linkIDs,2)
               p=[p; obj.m_linkProb(linkIDs(i))];
           end
       end

       function p = getLogLinkProbabilities( obj, linkIDs )
           p =[];
           for i=1:size(linkIDs,2)
               p=[p; obj.m_logLinkProb(linkIDs(i))];
           end
       end
       
       % this can be called only for links that are in the spanning tree
       function changeIDs  = getConnectedLinkIDs( obj, linkID ) %returns the linkIDs that change if linkID is changed
           movingVertex = obj.getMovingVertexInd( linkID )
           changeIDs = obj.m_vertexLinkIDs(movingVertex);
       end
      
       function newVertex = sampleVertex( obj, linkID, changingVertexInd )
           %sample the link
           origoState = obj.m_states( :, obj.m_linkIDmap(2,linkID) );
           linkListIndex = obj.m_linkIDmap(4,linkID);
           if( obj.m_linkIDmap(1,linkID) == 0 ) %state link
               x0 = obj.m_states( :, obj.m_linkIDmap(3,linkID) );
               x  = obj.m_stateStateLinkList( linkListIndex ).sampleLink( origoState, obj.m_dynamicalModel );
               
               if( changingVertexInd == obj.m_stateIndexMap( obj.m_linkIDmap(3,linkID) ) )%ez szar!
                   newVertex = x; 
               else 
                   newVertex = obj.m_states( :, obj.m_linkIDmap(2,linkID) ) + x0 - x;
               end
%               p  = obj.m_stateStateLinkList( obj.m_linkIDmap(2,linkID) ).getLinkProbability( origoState, x, obj.m_dynamicalModel );
           else  %state-feature link
               fID = obj.m_linkIDmap(3,linkID);
               f0  = obj.m_featureCoordinates( :, obj.m_featureIndexSeen(fID,2) );
               f   = obj.m_featureStateLinkList( linkListIndex ).sampleLink( origoState, obj.m_observationModel );               
                
               T = [f-f0;0];
               if( changingVertexInd == obj.m_featureIndexMap( obj.m_linkIDmap(3,linkID) ) ) %ez szar!
                   newVertex = f; 
               else %f0 stays
                   newVertex = obj.m_states( :, obj.m_linkIDmap(2,linkID) ) - T;
               end
%               p   = obj.m_featureStateLinkList( linkListIndex ).getLinkProbability( origoState, f, obj.m_observationModel );
           end           
       end
       
       function [rotationOrigo, T, fT, p] = sampleLinkTransformation( obj, linkID ) %samples a link and updates graph consistency accordingly
           %sample the link
           origoState = obj.m_states( :, obj.m_linkIDmap(2,linkID) );
           linkListIndex = obj.m_linkIDmap(4,linkID);
           if( obj.m_linkIDmap(1,linkID) == 0 ) %state link
               x0 = obj.m_states( :, obj.m_linkIDmap(3,linkID) );
               x  = obj.m_stateStateLinkList( linkListIndex ).sampleLink( origoState, obj.m_dynamicalModel );
               T  = x-x0; 
               rotationOrigo = [x(1), x(2)];
               fT = [0;0];
               p  = obj.m_stateStateLinkList( obj.m_linkIDmap(2,linkID) ).getLinkProbability( origoState, x, obj.m_dynamicalModel );
           else  %state-feature link
               fID = obj.m_linkIDmap(3,linkID);
               f0  = obj.m_featureCoordinates( :, obj.m_featureIndexSeen(fID,2) );
               f   = obj.m_featureStateLinkList( linkListIndex ).sampleLink( origoState, obj.m_observationModel );               
               fT   = f-f0;  
               T = [0,0,0];
               rotationOrigo = [0, 0];
               p   = obj.m_featureStateLinkList( linkListIndex ).getLinkProbability( origoState, f, obj.m_observationModel );
           end
       end


       function [logP, p] = getTransformedLinkProbabilities( obj, changingVertexInd, newVertex, linkIDs )
           p =[];
           logP = [];
           for i=1:size(linkIDs,2)
               linkID = linkIDs(i);
               linkListIndex = obj.m_linkIDmap(4,linkID);               
               
               if( obj.m_linkIDmap(1,linkID) == 1 ) %state-feature link 
                   x0 = obj.m_states( :, obj.m_linkIDmap(2,linkID) ); %theoretically this state changes
                   fID  = obj.m_linkIDmap(3,linkID);
                   f    = obj.m_featureCoordinates( :, obj.m_featureIndexSeen(fID,2) );

                   if( obj.m_stateIndexMap( obj.m_linkIDmap(2,linkID) ) == changingVertexInd )%ez szar!
                       prob = obj.m_featureStateLinkList( linkListIndex ).getLinkProbability( newVertex, f, obj.m_observationModel );
                       logProb = obj.m_featureStateLinkList( linkListIndex ).getLogLinkProbability( newVertex, f, obj.m_observationModel );               
                   else
                       prob = obj.m_featureStateLinkList( linkListIndex ).getLinkProbability( x0, newVertex, obj.m_observationModel );
                       logProb = obj.m_featureStateLinkList( linkListIndex ).getLogLinkProbability( x0, newVertex, obj.m_observationModel );               
                   end
               else  %state-state link
                   x0 = obj.m_states( :, obj.m_linkIDmap(2,linkID) );
                   x1 = obj.m_states( :, obj.m_linkIDmap(3,linkID) );
                   if( obj.m_stateIndexMap( obj.m_linkIDmap(2,linkID) ) == changingVertexInd )%ez szar!                       
                       prob = obj.m_stateStateLinkList( linkListIndex ).getLinkProbability( newVertex, x1, obj.m_dynamicalModel );
                       logProb = obj.m_stateStateLinkList( linkListIndex ).getLogLinkProbability( newVertex, x1, obj.m_dynamicalModel );               
                   else
                       prob = obj.m_stateStateLinkList( linkListIndex ).getLinkProbability( x0, newVertex, obj.m_dynamicalModel );
                       logProb = obj.m_stateStateLinkList( linkListIndex ).getLogLinkProbability( x0, newVertex, obj.m_dynamicalModel );               
                   end                   
               end
               p=[p; prob];
               logP=[logP; logProb];
           end
       end
       
       function obj = setVertex( objIn , linkID, changingVertexInd, newVertex, changeLinkIDs, linkLogP, linkP )
           obj       = objIn;
           %change the vertex
           if( obj.m_linkIDmap(1,linkID) == 0 ) %state link
               if( changingVertexInd == obj.m_stateIndexMap( obj.m_linkIDmap(3,linkID) ) )%ez szar!
                   obj.m_states( :, obj.m_linkIDmap(3,linkID) ) = newVertex; 
               else
                   obj.m_states( :, obj.m_linkIDmap(2,linkID) ) = newVertex; 
               end
           else  %state-feature link
               if( changingVertexInd == obj.m_featureIndexMap( obj.m_linkIDmap(3,linkID) ) ) %ez szar!
                   fID = obj.m_linkIDmap(3,linkID);
                   obj.m_featureCoordinates( :, obj.m_featureIndexSeen(fID,2) ) = newVertex; 
               else
                   obj.m_states( :, obj.m_linkIDmap(2,linkID) ) = newVertex; 
               end
           end           
          
           %change the probabilities
           for i=1:size(changeLinkIDs) 
               obj.m_linkProb(changeLinkIDs(i)) = linkP(i); 
               obj.m_logLinkProb(changeLinkIDs(i)) = linkLogP(i);
           end
           %change the graph edge directions
           obj.m_vertexMovedTime( changingVertexInd) = obj.m_stepNo;
           obj.m_stepNo = obj.m_stepNo + 1;
       end

       function obj = setEdgeDirections( objIn, changingVertexInd )
           obj       = objIn;
           %change the graph edge directions
           obj.m_vertexMovedTime( changingVertexInd ) = obj.m_stepNo;
           obj.m_stepNo = obj.m_stepNo + 1;
       end
       
       function obj = applyTransformation( objIn, rotationOrigo, T, fT, coreLinkID )
           obj       = objIn;
           % changing the core link
           coreLinkListIndex = obj.m_linkIDmap(4,coreLinkID);
           origoState = obj.m_states( :, obj.m_linkIDmap(2,coreLinkID) );
           if( obj.m_linkIDmap(1,coreLinkID) == 0 ) %state link
               stateIndex = obj.m_linkIDmap(3,coreLinkID);
               x = obj.m_states( :, stateIndex ) + T;
               obj.m_states( :, stateIndex ) = x;
               obj.m_linkProb(coreLinkID) = obj.m_stateStateLinkList( coreLinkListIndex ).getLinkProbability( origoState, x, obj.m_dynamicalModel );
               obj.m_logLinkProb(coreLinkID) = obj.m_stateStateLinkList( coreLinkListIndex ).getLogLinkProbability( origoState, x, obj.m_dynamicalModel );               
           else  %state-feature link
               fID = obj.m_linkIDmap(3,coreLinkID);
               f  = obj.m_featureCoordinates( :, obj.m_featureIndexSeen(fID,2) ) + fT;
               obj.m_linkProb(coreLinkID) = obj.m_featureStateLinkList( coreLinkListIndex ).getLinkProbability( origoState, f, obj.m_observationModel );
               obj.m_logLinkProb(coreLinkID) = obj.m_featureStateLinkList( coreLinkListIndex ).getLogLinkProbability( origoState, f, obj.m_observationModel );
           end
                       
           %changing related link probabilities;
           changeIDs = obj.getConnectedLinkIDs( coreLinkID );
           p = getTransformedLinkProbabilities( obj, rotationOrigo, T, fT, changeIDs );           
           for i=1:size(changeIDs,1)
               linkID = changeIDs(i);
               linkListIndex = obj.m_linkIDmap(4,linkID);
               if( obj.m_linkIDmap(1, linkID) == 0 ) % state-state link
                   x0 = obj.m_states( :, obj.m_linkIDmap(2,linkID) );
                   x1 = obj.m_states( :, obj.m_linkIDmap(3,linkID) );
                   obj.m_linkProb(linkID) = obj.m_stateStateLinkList( linkListIndex ).getLinkProbability( x0, x1, obj.m_dynamicalModel );
                   obj.m_logLinkProb(linkID) = obj.m_stateStateLinkList( linkListIndex ).getLogLinkProbability( x0, x1, obj.m_dynamicalModel );
               else %state-feature link
                   x = obj.m_states( :, obj.m_linkIDmap(2,linkID) );
                   fID = obj.m_linkIDmap(3,linkID);
                   f  = obj.m_featureCoordinates( :, obj.m_featureIndexSeen(fID,2) );
                   obj.m_linkProb(linkID) = obj.m_featureStateLinkList( linkListIndex ).getLinkProbability( x, f, obj.m_observationModel );
                   obj.m_logLinkProb(linkID) = obj.m_featureStateLinkList( linkListIndex ).getLogLinkProbability( x, f, obj.m_observationModel );
               end               
           end

       end
       
       function stateCoordinates = getState( obj )
           stateCoordinates = obj.m_states;
       end

       function featureCoordinates = getFeatureCoordinates( obj )
           featureCoordinates = obj.m_featureCoordinates;
       end
       
       function logLikelihood = getFullLogLikelihood( obj )
           logLikelihood = 0;
           for i=1:size(obj.m_linkIDmap,2)
               logLikelihood = logLikelihood + obj.m_logLinkProb(i);
           end
       end

       function stateLinkLogLikelihood        = getStateLinkLogLikelihood( obj )
           stateLinkLogLikelihood = [];
           for linkID=1:size(obj.m_linkIDmap,2)
               if( obj.m_linkIDmap(1, linkID) == 0 ) % state-state link
                   stateLinkLogLikelihood = [stateLinkLogLikelihood, obj.m_logLinkProb(linkID)];
               end               
           end
       end
       
       function stateFeatureLinkLogLikelihood = getStateFeatureLinkLogLikelihood( obj )
           stateFeatureLinkLogLikelihood = [];           
           for linkID=1:size(obj.m_linkIDmap,2)
               if( obj.m_linkIDmap(1, linkID) == 1 ) % state-state link
                   stateFeatureLinkLogLikelihood = [stateFeatureLinkLogLikelihood, obj.m_logLinkProb(linkID)];
               end               
           end
       end
       
       
       %the spanning tree links are drawn with a different color
       function drawFullGraph(obj, lm, wp, s1, s2)
%           plot(lm(1,:),lm(2,:),'g*')
           hold on, axis equal
%           plot(wp(1,:),wp(2,:), wp(1,:),wp(2,:),'ro')
           
           %draw all links 
           for linkID=1:size(obj.m_linkIDmap,2)
               stateIndex0 = obj.m_linkIDmap(2,linkID);
               x0 = obj.m_states( :, stateIndex0 );
               if( obj.m_linkIDmap(1,linkID) == 0 ) %state link
                   stateIndex1 = obj.m_linkIDmap(3,linkID);
                   x1 = obj.m_states( :, stateIndex1 );
                   plot( [x0(1), x1(1)], [x0(2), x1(2)], s1, 'LineWidth',2 );
               else %state-feature link
                   fID = obj.m_linkIDmap(3,linkID);
                   f  = obj.m_featureCoordinates( :, obj.m_featureIndexSeen(fID,2) );
                   plot( [x0(1), f(1)], [x0(2), f(2)], s2 );
               end
           end
           
           %re-draw spanning tree links with a different color
           for i=1:size(obj.m_spanningTreeLinkIDList,2)
               linkID = obj.m_spanningTreeLinkIDList(i);
               stateIndex0 = obj.m_linkIDmap(2,linkID);
               x0 = obj.m_states( :, stateIndex0 );
               if( obj.m_linkIDmap(1,linkID) == 1 ) %state-feature link
                   fID = obj.m_linkIDmap(3,linkID);
                   f  = obj.m_featureCoordinates( :, obj.m_featureIndexSeen(fID,2) );
                   plot( [x0(1), f(1)], [x0(2), f(2)], s1, 'LineWidth',2  );
               end
           end
       end

       function drawChangingLinks( obj, coreLinkID )
           hold on, axis equal

           %draw all links 
           for linkID=1:size(obj.m_linkIDmap,2)
               stateIndex0 = obj.m_linkIDmap(2,linkID);
               x0 = obj.m_states( :, stateIndex0 );
               if( obj.m_linkIDmap(1,linkID) == 0 ) %state link
                   stateIndex1 = obj.m_linkIDmap(3,linkID);
                   x1 = obj.m_states( :, stateIndex1 );
                   plot( [x0(1), x1(1)], [x0(2), x1(2)], 'g-' );
               else %state-feature link
                   fID = obj.m_linkIDmap(3,linkID);
                   f  = obj.m_featureCoordinates( :, obj.m_featureIndexSeen(fID,2) );
                   plot( [x0(1), f(1)], [x0(2), f(2)], 'g-' );
               end
           end
           
           %draw the core Link
           stateIndex0 = obj.m_linkIDmap(2,coreLinkID);
           x0 = obj.m_states( :, stateIndex0 );
           if( obj.m_linkIDmap(1,coreLinkID) == 0 ) %state link
               stateIndex1 = obj.m_linkIDmap(3,coreLinkID);
               x1 = obj.m_states( :, stateIndex1 );
               plot( [x0(1), x1(1)], [x0(2), x1(2)], 'r-' );
           else %state-feature link
               fID = obj.m_linkIDmap(3,coreLinkID);
               f  = obj.m_featureCoordinates( :, obj.m_featureIndexSeen(fID,2) );
               plot( [x0(1), f(1)], [x0(2), f(2)], 'r-' );
           end
           
           %draw the changing links
           changeIDs  = obj.getConnectedLinkIDs( coreLinkID );
           for i=1:size(changeIDs,2)
               linkID = changeIDs(i);
               stateIndex0 = obj.m_linkIDmap(2,linkID);
               x0 = obj.m_states( :, stateIndex0 );
               if( obj.m_linkIDmap(1,linkID) == 0 ) %state link
                   stateIndex1 = obj.m_linkIDmap(3,coreLinkID);
                   x1 = obj.m_states( :, stateIndex1 );
                   plot( [x0(1), x1(1)], [x0(2), x1(2)], 'b-' );
               else %state-feature link
                   fID = obj.m_linkIDmap(3,linkID);
                   f  = obj.m_featureCoordinates( :, obj.m_featureIndexSeen(fID,2) );
                   plot( [x0(1), f(1)], [x0(2), f(2)], 'b-' );
               end           
           end
       end
   end
end 
