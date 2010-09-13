function [stateCoordinates, featureCoordinates, featureIdx] = getAllSLAMstates( particles, da_table )
    s = size(particles(1).xf,2);
    featureIdx = zeros( 1, s );
    for i=1:s
        featureIdx(i) = find( da_table==i );
    end

    NPARTICLES = size( particles,2 );
    crapped = 0;
    for i=1:NPARTICLES
        if particles(i).w > 1 || particles(i).w < 0
            crapped = 1;
        end
    end
    if crapped == 1
        for i=1:NPARTICLES
            particles(i).w = 1/NPARTICLES;
        end    
    end
        
    
    stateCoordinates = particles(1).w*particles(1).xvHist;
    featureCoordinates = particles(1).w*particles(1).xf;
    for i=1:NPARTICLES
        stateCoordinates = stateCoordinates + particles(i).w*particles(i).xvHist;
        featureCoordinates = featureCoordinates + particles(i).w*particles(i).xf;    
    end
end