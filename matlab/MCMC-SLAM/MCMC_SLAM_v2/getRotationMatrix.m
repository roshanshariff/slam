function R = getRotationMatrix( theta )
    R = zeros(2,2);
    R(1,:) = [cos( theta ), -sin( theta )];
    R(2,:) = [sin( theta ),  cos( theta )];
end