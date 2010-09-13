classdef MCMC_SLAM < handle

   properties 
       m_SLAMgraph;
       
       m_dynamicalModel;
       
       m_observationModel;
       
       m_MLstates;                       % the state coordinate realizations (including facing \theta)
       m_MLfeatureCoordinates;           % the feature coordinate ralizations

   end
   
   methods
       function obj = MCMC_SLAM( Vpar, Qpar, dt_controls, wheelBase, Rpar )
           obj.m_dynamicalModel = DynamicalModel( Vpar, Qpar, dt_controls, wheelBase );
           obj.m_observationModel = ObservationModel( Rpar );
           obj.m_SLAMgraph = SLAMgraph( obj.m_dynamicalModel, obj.m_observationModel );
       end

       function obj = addState( objIn, Control )
           obj             = objIn;
           obj.m_SLAMgraph = obj.m_SLAMgraph.addState( Control );
       end
       
       function [obj, firstSeenFeatureIdx] = addObservedState( objIn, Control, zVec )
           obj             = objIn;
           [obj.m_SLAMgraph, firstSeenFeatureIdx] = obj.m_SLAMgraph.addObservedState( Control, zVec );
       end
       
       function [obj,logLikelihoodHistory] = step( objIn, iterNo )
           obj = objIn;
           logLikelihoodHistory = [];
           maxLogLike = 1000;
           for i = 1:iterNo
               obj = MCMCstep( obj );
               fullLogLike = obj.getFullLogLikelihood();
               logLikelihoodHistory = [logLikelihoodHistory, fullLogLike];
               if i==1 || maxLogLike < fullLogLike
                   maxLogLike = fullLogLike;
                   obj.m_MLstates = obj.m_SLAMgraph.getState();
                   obj.m_MLfeatureCoordinates = obj.m_SLAMgraph.getFeatureCoordinates();
               end    
           end
       end
       
       function stateCoordinates = getState( obj )
           stateCoordinates = obj.m_SLAMgraph.getState();
       end

       function featureCoordinates = getFeatureCoordinates( obj )
           featureCoordinates = obj.m_SLAMgraph.getFeatureCoordinates();
       end

       function stateCoordinates = getMLState( obj )
           stateCoordinates = obj.m_MLstates;
       end

       function featureCoordinates = getMLFeatureCoordinates( obj )
           featureCoordinates = obj.m_MLfeatureCoordinates;
       end
       
       function logLikelihood = getFullLogLikelihood( obj )
           logLikelihood = obj.m_SLAMgraph.getFullLogLikelihood();
       end
       
       function stateLinkLogLikelihood        = getStateLinkLogLikelihood( obj )
           stateLinkLogLikelihood = obj.m_SLAMgraph.getStateLinkLogLikelihood();
       end
       
       function stateFeatureLinkLogLikelihood = getStateFeatureLinkLogLikelihood( obj )
           stateFeatureLinkLogLikelihood = obj.m_SLAMgraph.getStateFeatureLinkLogLikelihood();           
       end
       
       function drawFullGraph(obj, lm, wp)
           figure
           obj.m_SLAMgraph.drawFullGraph( lm, wp, 'b-', 'r-'  );
       end
       
       function drawChangingLinks( obj, coreLinkID )
           figure
           obj.m_SLAMgraph.drawChangingLinks( coreLinkID );
       end

       function drawMove( obj, lm, wp, coreLinkID )
           figure
           obj.m_SLAMgraph.drawFullGraph( lm, wp, 'b-', 'r-' );
           obj.MCMCstep();
           obj.m_SLAMgraph.drawFullGraph( lm, wp, 'g-', 'y-'  );           
       end
   end
   
   methods ( Access = private )       
       function  obj = MCMCstep( objIn )
           obj = objIn;
           % draw a proposal link index in the spanning tree
           linkIDs = obj.m_SLAMgraph.getSpanningTreeLinkIDs();
           p = obj.m_SLAMgraph.getLinkProbabilities( linkIDs );
           dens  = [];
           alpha = 0;
           for i = 1:size(p)
               dens  = [dens, 1/p(i)];
               alpha = alpha + 1/p(i);
           end
           ind = logSampling( dens );
           linkID = linkIDs(ind);

           pLink       = obj.m_SLAMgraph.getLinkProbabilities( linkID );
           changeIDs   = obj.m_SLAMgraph.getConnectedLinkIDs( linkID );
           logChangeP     = obj.m_SLAMgraph.getLogLinkProbabilities( changeIDs );
           sumLogChangeP = 0;
           for j = 1:size(logChangeP)
               sumLogChangeP = sumLogChangeP + logChangeP(j);
           end
           
           [rotationOrigo, T, featureT, pLinkTemp] = obj.m_SLAMgraph.sampleLinkTransformation( linkID );
           alphaTemp       = alpha - 1/pLink + 1/pLinkTemp;
           [logChangePTemp, changePTemp] = obj.m_SLAMgraph.getTransformedLinkProbabilities( rotationOrigo, T, featureT, changeIDs );

           sumLogChangePTemp = 0;
           for j = 1:size(logChangePTemp)
               sumLogChangePTemp = sumLogChangePTemp + logChangePTemp(j);
           end
           
           changeRate = exp( sumLogChangePTemp - sumLogChangeP );
           
           % calculate acceptance probability
           if( alphaTemp*pLinkTemp > 0 )
%               A = (prodChangePTemp/prodChangeP)*(alpha/alphaTemp)*(pLink/pLinkTemp);
%               A = changeRate*(alpha/alphaTemp)*(pLink/pLinkTemp);
               A = changeRate*(alpha/alphaTemp)*(pLinkTemp/pLink);
           else
               for j = 1:size(logChangeP)
                   logChangeP(j)
               end
               pLink
               for j = 1:size(logChangePTemp)
                   logChangePTemp(j)
               end
               A = 1;
           end
           % do the MCMC move (change temporal state to regular)
           if( rand(1,1) < A )
               obj.m_SLAMgraph = obj.m_SLAMgraph.applyTransformation( rotationOrigo, T, featureT, linkID );               
           end
       end
   end
end