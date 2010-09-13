classdef GraphLink 

   properties
       type;  %state-state link : 0; feature-state link : 1
       link;  %the link itself
       fromLinkID; % negative if this is a feature point
       toLinkID; % negative if this is a feature point
   end

   methods
       function obj = GraphLink( typePar, initPar, fromLinkIDpar, toLinkIDpar )
           obj.fromLinkID = fromLinkIDpar;
           obj.toLinkID = toLinkIDpar;
           obj.type = typePar;
           if( obj.type == 0 )
               obj.link = StateStateLink( initPar );
           else
               obj.link = FeatureStateLink( initPar );               
           end
       end
       
       function x = linkSample( obj )
           x = obj.link.linkSample();
       end
       
       function p = getLinkProbability( obj, dx )
           p = obj.link.getLinkProbability( dx );
       end
   end
end 
