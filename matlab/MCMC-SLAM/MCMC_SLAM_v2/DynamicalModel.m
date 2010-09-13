classdef DynamicalModel < handle
%SLAM_DYN Summary of this class goes here
%   Detailed explanation goes here

   properties
       V;
       Q;
       DT_CONTROLS;
       WHEELBASE;
   end

   methods
       function obj = DynamicalModel( Vpar, Qpar, dt_controls, wheelBase  )
           obj.V            = Vpar;
           obj.Q            = Qpar;
           obj.DT_CONTROLS  = dt_controls;
           obj.WHEELBASE    = wheelBase;
       end
       
       function x = dynSample( obj, xprev, Control )
           C= multivariate_gauss([obj.V;Control],obj.Q, 1); 
           Vn= C(1);
           Gn= C(2);
%           x= predict_true(xprev, Vn, Gn, obj.WHEELBASE, obj.DT_CONTROLS);
           x= [xprev(1) + Vn*obj.DT_CONTROLS*cos(Gn+xprev(3,:)); 
               xprev(2) + Vn*obj.DT_CONTROLS*sin(Gn+xprev(3,:));
               xprev(3) + Vn*obj.DT_CONTROLS*sin(Gn)/obj.WHEELBASE];

       end
       
       function p = getProbability( obj, xprev, x, Control )
           dx = x-xprev;
           Vn  = (dx(1)^2+dx(2)^2)^0.5/obj.DT_CONTROLS;
           a   = dx(3)*obj.WHEELBASE/(Vn*obj.DT_CONTROLS);
           %TODO: what if a<-1 or a>1 possible bug here!
           Gn  = asin( a );
           %xx  = predict_true(xprev, Vn, Gn, obj.WHEELBASE, obj.DT_CONTROLS)
           at  = [Vn-obj.V;Gn-Control];
           p  = gauss_likelihood(at,obj.Q,0);
       end

       function p = getLogLikelihood( obj, xprev, x, Control )
           dx = x-xprev;
           Vn  = (dx(1)^2+dx(2)^2)^0.5/obj.DT_CONTROLS;
           a   = dx(3)*obj.WHEELBASE/(Vn*obj.DT_CONTROLS);
           %TODO: what if a<-1 or a>1 possible bug here!
           Gn  = asin( a );
           %xx  = predict_true(xprev, Vn, Gn, obj.WHEELBASE, obj.DT_CONTROLS)
           at  = [Vn-obj.V;Gn-Control];
           p  = gauss_likelihood(at,obj.Q,1);
       end
   end
end 
