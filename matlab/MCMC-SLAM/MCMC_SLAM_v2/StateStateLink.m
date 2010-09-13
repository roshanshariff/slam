classdef StateStateLink < handle

   properties
       m_control;  %the control parameter
       m_dynamicalModel;
   end

   methods
       function obj = StateStateLink( ControlPar, dynamicalModel )
           obj.m_control = ControlPar;
           obj.m_dynamicalModel = dynamicalModel;
       end
       
       function x = sampleLink( obj )
           xprev = zeros(3,1);
           x = obj.m_dynamicalModel.dynSample( xprev, obj.m_control );
       end
       
       function p = getLinkProbability( obj, x )
           xprev = zeros(3,1);
           p = obj.m_dynamicalModel.getProbability( xprev, x, obj.m_control );
       end

       function p = getLogLinkProbability( obj, x )
           xprev = zeros(3,1);
           p = obj.m_dynamicalModel.getLogLikelihood( xprev, x, obj.m_control );
       end
   end
end 
