classdef FeatureStateLink < handle

   properties
       m_obsZ;                   % the control parameter
       m_observationModel;       % the observationModel
   end

   methods
       function obj = FeatureStateLink( obsZ, observationModel )
           obj.m_obsZ = obsZ;
           obj.m_observationModel = observationModel;
       end
       
       function f = sampleLink( obj )
           x = zeros(3,1);
           f = obj.m_observationModel.obsSample( x, obj.m_obsZ );
           f = [f;0]; %dummy dimension to make everything 3 dimensional
       end
       
       function p = getLinkProbability( obj, f )
           x = zeros(3,1);
           p = obj.m_observationModel.getProbability( x, f, obj.m_obsZ );
       end

       function p = getLogLinkProbability( obj, f )
           x = zeros(3,1);
           p = obj.m_observationModel.getLogLikelihood( x, f, obj.m_obsZ );
       end
   end
end 
