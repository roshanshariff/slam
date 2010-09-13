% we return the transformation between the two states
function transformation = getTransformation( origState, newState )
    %remark: if the transformation was initiated by a feature translation
    %than T(3)=0, so there is no rotation, so the rotation origo can be
    %anything
	t = newState-origState; 
    %we need the transformation in the new coordinate system, this is why
    %we need to rotate it with transformation(3)
    transformation = t;
    transformation(1) = cos( -transformation(3) )*t(1) - sin( -transformation(3) )*t(2); 
    transformation(2) = sin( -transformation(3) )*t(1) + cos( -transformation(3) )*t(2); 

    
end


