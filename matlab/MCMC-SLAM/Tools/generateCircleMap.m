function [lm,wp]=generateCircleMap(r)
    lm = [];
    for i=0:25:2*r+25
        for j=-2*r:25:2*r+25
            lm = [lm,[i;j]];
        end
    end

    figure(42);
    plot(lm(1,:),lm(2,:),'k*')
    
    wp = [];
    x0 = r;
    y0 = 0;
    for i=-10:10
        wp = [wp, [r*cos(i*pi/10)+x0; r*sin(i*pi/10)+y0]];
    end
    axis([-10 2*r+10 -r-10 r+10])
    xlim([-10 2*r+10])
    ylim([-r-10 r+10])
    hold on, %axis equal
    plot(wp(1,:),wp(2,:), wp(1,:),wp(2,:),'b', 'LineWidth', 2)
end