index = index + 1

set output prefix.index.'.gif'

plot prefix.'landmarks.txt' using 1:2 title 'Landmarks' ls 1, \
     prefix.'landmarks.estimated.'.index.'.txt' using 1:2 title 'Estimated Landmarks' ls 2, \
     prefix.'trajectory.estimated.'.index.'.txt' using 1:2 with lines title 'Estimated Trajectory' ls 3, \
     prefix.'trajectory.txt' every ::::index using 1:2 with lines title 'Trajectory' ls 4

unset output

if (index < steps) reread
