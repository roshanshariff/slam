index = index + 1

set output prefix.index.'.gif'

plot prefix.'landmarks.txt' using 1:2 title 'Landmarks' ls 1, \
     prefix.'landmarks.estimated.'.index.'.txt' using 1:2 title 'Estimated Landmarks' ls 2, \
     prefix.'trajectory.expected.txt' every ::::index using 1:2 with lines title 'Measured Trajectory' ls 3, \
     prefix.'trajectory.expected.txt' every ::index::index using 1:2:(5.0*cos($3)):(5.0*sin($3)) notitle with vectors as 4, \
     prefix.'trajectory.estimated.'.index.'.txt' using 1:2 with lines title 'Estimated Trajectory' ls 5, \
     prefix.'trajectory.estimated.'.index.'.txt' every ::index::index using 1:2:(5.0*cos($3)):(5.0*sin($3)) notitle with vectors as 6, \
     prefix.'trajectory.txt' every ::::index using 1:2 with lines title 'Actual Trajectory' ls 7, \
     prefix.'trajectory.txt' every ::index::index using 1:2:(5.0*cos($3)):(5.0*sin($3)) notitle with vectors as 8

unset output

if (index < steps) reread
