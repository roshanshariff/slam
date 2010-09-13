classdef ObservationModel < handle
%ObservationModel Summary of this class goes here
%   Detailed explanation goes here

   properties
       R;
   end

   methods
       function obj = ObservationModel( Rpar )
           obj.R   = Rpar;
       end
       
       function f = obsSample( obj, x, obsZ )
           z(1)= obsZ(1,1) + randn(1,1)*sqrt(obj.R(1,1));
           z(2)= obsZ(1,2) + randn(1,1)*sqrt(obj.R(2,2));
        
           f = zeros(2,1);
           f(1) = x(1) + z(1)*cos(x(3) + z(2));
           f(2) = x(2) + z(1)*sin(x(3) + z(2));
       end
       
       function p = getProbability( obj, x, f, obsZ )
           z = zeros(1,2);
           z(1,1) = ( (f(1)-x(1))^2 + (f(2)-x(2))^2 )^0.5;
           z(1,2) = atan2( (f(2)-x(2))/z(1,1), (f(1)-x(1))/z(1,1) ) - x(3);
           dz = z-obsZ;
           dz(2) = pi_to_pi( dz(2) );
           p  = gauss_likelihood(dz(1),obj.R(1,1),0) * gauss_likelihood(dz(2),obj.R(2,2),0);
       end

       function p = getLogLikelihood( obj, x, f, obsZ )
           z = zeros(1,2);
           z(1,1) = ( (f(1)-x(1))^2 + (f(2)-x(2))^2 )^0.5;
           z(1,2) = atan2( (f(2)-x(2))/z(1,1), (f(1)-x(1))/z(1,1) ) - x(3);
           dz = z-obsZ;
           dz(2) = pi_to_pi( dz(2) );
%           if( pi+dz(2) > 2*pi-(pi+dz(2) ) % we take the smaller angle! ;
%           but gauss is simmetrical
%               dz(2) = -dz(2);
%           end
           p  = gauss_likelihood(dz(1),obj.R(1,1),1) + gauss_likelihood(dz(2),obj.R(2,2),1);
       end
   end
end 
