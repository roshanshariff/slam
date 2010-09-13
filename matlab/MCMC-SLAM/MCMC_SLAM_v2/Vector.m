classdef Vector < handle
   properties
       v;  
   end

   methods       
       function obj = Vector()
           obj.v = [];
       end
       
       function s = vsize(obj)
           s = size(obj.v,2);
       end
       
       function push_back( obj, elem )
           obj.v = [obj.v, elem];
       end
       
       function e = at( obj, index )
           e = obj.v( index );
       end
   end
end 
