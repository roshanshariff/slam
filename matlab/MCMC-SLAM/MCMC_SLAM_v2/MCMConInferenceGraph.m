classdef MCMConInferenceGraph < handle

   properties 
       m_inferenceGraphRealization;     % the realization is the state of the Markov Chain 
           
       m_inferenceGraphProposal;        % the generated proposal that wil or will not be accepted       
       
       m_spanningTreeRealization;       % the spanning tree used to hold the absolute coordinates
       
       %debug
       m_sumLogChangeProbDeltaVec;
   end
   
   methods
       function obj = MCMConInferenceGraph( inferenceGraph )
            obj.m_inferenceGraphRealization = InferenceGraphRealization( inferenceGraph );
            obj.m_inferenceGraphProposal    = InferenceGraphProposal( obj.m_inferenceGraphRealization );
            obj.m_sumLogChangeProbDeltaVec  = [];
            obj.m_spanningTreeRealization = SpanningTreeRealization( obj.m_inferenceGraphRealization );
       end
       
       function obj = step( objIn, iterNo )
           obj = objIn;
           obj.m_spanningTreeRealization = SpanningTreeRealization( obj.m_inferenceGraphRealization );
 
           
           %checking graph realization consistency
%           ig = obj.m_inferenceGraphRealization.m_inferenceGraph;
%           sumErr = 0;
%           for i=1:size(obj.m_inferenceGraphRealization.m_edgeRealizations,2)
%               edge = ig.m_edges(i,:);
%               edgeInd1 = find( ig.m_stateIDs == edge(1) );
%               edgeInd2 = find( ig.m_stateIDs == edge(2) );
%               
%               n1 = obj.m_spanningTreeRealization.m_nodeRealizations( :, edgeInd1);
%               n2 = obj.m_spanningTreeRealization.m_nodeRealizations( :, edgeInd2);
%               realization = obj.m_inferenceGraphRealization.m_edgeRealizations( :, i );
%               v=zeros(3,1);
%               theta = n1(3);
%               v(1) = cos( theta )*realization(1) - sin( theta )*realization(2);
%               v(2) = sin( theta )*realization(1) + cos( theta )*realization(2);
%               if( edge(2) > 0 )
%                   v(3) = realization(3);
%               end
%               nn2 = n1+v;
%               if( edge(2) < 0 )
%                   nn2(3) = 0;
%               end
               
%               err = sum(abs(nn2-n2));
%               if( err > 0.0001 )
%                   edge
%                   err
%                   n2
%                   nn2
%               end
%               sumErr = sumErr+err;
%           end
           
%           if sumErr > 0.001
%               sumErr
%           end
           
           for i = 1:iterNo
               obj = MCMCstep( obj );
           end
       end              
   end
   
   methods ( Access = private )       
       function  obj = MCMCstep( objIn )
           obj = objIn;
           [obj.m_inferenceGraphProposal, sumLogChangeProbDelta, origoStateInd, transformation ] = obj.m_inferenceGraphProposal.propose( obj.m_spanningTreeRealization );
           
           alpha              = obj.m_inferenceGraphRealization.getAlpha();
           proposalAlpha      = alpha + obj.m_inferenceGraphProposal.getAlphaDiff();
           coreChangeLikeRate = obj.m_inferenceGraphProposal.getCoreChangeLikeRate();
           changeRate         = exp( sumLogChangeProbDelta );
           
           % calculate acceptance probability
           if( proposalAlpha > 0 )
               A = changeRate*(alpha/proposalAlpha)*coreChangeLikeRate;
           else
               'Numerical Problem at MCMC'
               A = 1;
           end
                      
           obj.m_sumLogChangeProbDeltaVec = [obj.m_sumLogChangeProbDeltaVec, sumLogChangeProbDelta];
%           figure(22);
%           plot( obj.m_sumLogChangeProbDeltaVec );
           
           % do the MCMC move (change temporal state to regular)
           if( rand(1,1) < A )
 %              [stateCoordinates0, featureCoordinates0] = getAllSLAMstates( obj.m_inferenceGraphRealization );
               obj.m_inferenceGraphProposal.accept();               
               obj.m_spanningTreeRealization = SpanningTreeRealization( obj.m_inferenceGraphRealization );
               %obj.m_spanningTreeRealization.setTransformation( origoStateInd, transformation );
%               [stateCoordinates1, featureCoordinates1] = getAllSLAMstates( obj.m_inferenceGraphRealization );
               
%               figure(2);
%               plot( stateCoordinates0(1,:),stateCoordinates0(2,:) );
%               plot( featureCoordinates0(1,:),featureCoordinates0(2,:), 'ro' );
%               hold on;
%               plot( stateCoordinates1(1,:),stateCoordinates1(2,:) );
%               plot( featureCoordinates1(1,:),featureCoordinates1(2,:), 'ro' );
           end
       end
   end
end