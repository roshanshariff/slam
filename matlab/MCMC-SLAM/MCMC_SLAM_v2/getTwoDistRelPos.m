function w = getTwoDistRelPos( v, u )
    R = getRotationMatrix( v(3) );
    w = [v(1:2)+R*u(1:2);u(3)+v(3)];
end


