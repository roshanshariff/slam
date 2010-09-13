classdef MCMC_SLAM_v2 < handle

   properties 
       m_SLAMgraph;
       
       m_dynamicalModel;
       
       m_observationModel;
   end
   
   methods
       function obj = MCMC_SLAM_v2( Vpar, Qpar, dt_controls, wheelBase, Rpar )
           obj.m_dynamicalModel = DynamicalModel( Vpar, Qpar, dt_controls, wheelBase );
           obj.m_observationModel = ObservationModel( Rpar );
           obj.m_SLAMgraph = SLAMgraph_v2( obj.m_dynamicalModel, obj.m_observationModel );
       end

       function obj = addState( objIn, Control )
           obj             = objIn;
           obj.m_SLAMgraph = obj.m_SLAMgraph.addState( Control );
       end
       
       function obj = addObservedState( objIn, Control, zVec )
           obj             = objIn;
           obj.m_SLAMgraph = obj.m_SLAMgraph.addObservedState( Control, zVec );
       end
       
       function [obj,logLikelihoodHistory] = step( objIn, iterNo )
           obj = objIn;
           logLikelihoodHistory = [];
           for i = 1:iterNo
               obj = MCMCstep( obj );
               logLikelihoodHistory = [logLikelihoodHistory, obj.getFullLogLikelihood()];
           end
       end
       
       function stateCoordinates = getState( obj )
           stateCoordinates = obj.m_SLAMgraph.getState();
       end

       function featureCoordinates = getFeatureCoordinates( obj )
           featureCoordinates = obj.m_SLAMgraph.getFeatureCoordinates();
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
           linkIDs = obj.m_SLAMgraph.getAllLinkIDs();
           p = obj.m_SLAMgraph.getLinkProbabilities( linkIDs );
           dens  = [];
           alpha = 0;
           for i = 1:size(p)
               dens  = [dens, 1/p(i)];
               alpha = alpha + 1/p(i);
           end
           ind = logSampling( dens );
           linkID = linkIDs(ind);

           changingVertexInd = obj.m_SLAMgraph.getMovingVertexInd( linkID );
           inLinkIDs         = obj.m_SLAMgraph.getInLinkIDs( changingVertexInd );
           outLinkIDs        = obj.m_SLAMgraph.getOutLinkIDs( changingVertexInd );

           inLinkLogP        = obj.m_SLAMgraph.getLogLinkProbabilities( inLinkIDs );
           outLinkLogP       = obj.m_SLAMgraph.getLogLinkProbabilities( outLinkIDs );
           inLinkP           = obj.m_SLAMgraph.getLinkProbabilities( inLinkIDs );
           outLinkP          = obj.m_SLAMgraph.getLinkProbabilities( outLinkIDs );
           
           newVertex         = obj.m_SLAMgraph.sampleVertex( linkID, changingVertexInd );
           [inLinkLogChangePTemp, inLinkChangePTemp]   = obj.m_SLAMgraph.getTransformedLinkProbabilities( changingVertexInd, newVertex, inLinkIDs );
           [outLinkLogChangePTemp, outLinkChangePTemp] = obj.m_SLAMgraph.getTransformedLinkProbabilities( changingVertexInd, newVertex, outLinkIDs );

           sumLogChangeP     = 0;
           sumLogChangePTemp = 0;
           alphaTemp         = alpha;
           sumTRate          = 0;
           sumTRateTemp      = 0;
           for j = 1:size(inLinkP)
               sumLogChangeP     = sumLogChangeP + inLinkLogP(j);
               sumLogChangePTemp = sumLogChangePTemp + inLinkLogChangePTemp(j);
               alphaTemp         = alphaTemp - 1/inLinkP(j) + 1/inLinkChangePTemp(j);
               sumTRate          = sumTRate + inLinkChangePTemp(j)/inLinkP(j);
               sumTRateTemp      = sumTRateTemp + inLinkP(j)/inLinkChangePTemp(j);
           end
           for j = 1:size(outLinkP)
               sumLogChangeP     = sumLogChangeP + outLinkLogP(j);
               sumLogChangePTemp = sumLogChangePTemp + outLinkLogChangePTemp(j);
               alphaTemp         = alphaTemp - 1/outLinkP(j) + 1/outLinkChangePTemp(j);
           end
                      
           changeRate = exp( sumLogChangePTemp - sumLogChangeP );
           
%           [sumRateNum, sumRateDenom]     = frac_SumFrac_ai_bi_SumFrac_bi_ai( inLinkP, inLinkChangePTemp );           
%           pTemp = obj.m_SLAMgraph.getLinkProbabilities( linkIDs );
%           [alphaRateNum, alphaRateDenom] = frac_SumFrac_1_ai_SumFrac_1_bi(p, pTemp); 

           % calculate acceptance probability
           if( alphaTemp*sumTRate > 0 )
%               A = (prodChangePTemp/prodChangeP)*(alpha/alphaTemp)*(pLink/pLinkTemp);
               A = changeRate*(alpha/alphaTemp)*(sumTRateTemp/sumTRate);
           else
               A = 1;
           end
           % do the MCMC move (change temporal state to regular)
           if( rand(1,1) < A )
               linkIDs  = [inLinkIDs, outLinkIDs];
               linkLogP = [inLinkLogChangePTemp; outLinkLogChangePTemp];
               linkP    = [inLinkChangePTemp; outLinkChangePTemp];
               obj.m_SLAMgraph = obj.m_SLAMgraph.setVertex( linkID, changingVertexInd, newVertex, linkIDs, linkLogP, linkP );               
           else %change the edge directions only
               obj.m_SLAMgraph = obj.m_SLAMgraph.setEdgeDirections( changingVertexInd );                              
           end
       end
   end
end