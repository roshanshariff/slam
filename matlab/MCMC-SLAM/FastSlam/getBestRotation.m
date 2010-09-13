%return the best theta for which R(theta)*lmVec0 ~ lmVec1
function dTheta = getBestRotation( lmVec0, lmVec1 )
    s = size( lmVec0,2 );
    sum_dTheta = 0;
    for i=1:s
        theta0 = atan2(lmVec0(2,i), lmVec0(1,i));
        theta1 = atan2(lmVec1(2,i), lmVec1(1,i));
        dTheta = theta1-theta0;
        if dTheta < -pi 
            dTheta = dTheta + 2*pi;
        elseif  dTheta > pi 
            dTheta = dTheta - 2*pi;
        end
        sum_dTheta = sum_dTheta + dTheta;
    end
    dTheta = sum_dTheta/s; 
end


