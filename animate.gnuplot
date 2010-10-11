index = index + 1

set output prefix.index.'.gif'

plot prefix.'landmarks.txt' using 1:2 title 'Landmarks' ls 1, \
     prefix.'landmarks.estimated.'.index.'.txt' using 1:2 title 'Estimated Landmarks' ls 2, \
     prefix.'state.expected.txt' every ::::index using 1:2 with lines title 'Measured State' ls 3, \
     prefix.'state.expected.txt' every ::index::index using 1:2:(5.0*cos($3)):(5.0*sin($3)) notitle with vectors as 4, \
     prefix.'state.estimated.'.index.'.txt' using 1:2 with lines title 'Estimated State' ls 5, \
     prefix.'state.estimated.'.index.'.txt' every ::index::index using 1:2:(5.0*cos($3)):(5.0*sin($3)) notitle with vectors as 6, \
     prefix.'state.txt' every ::::index using 1:2 with lines title 'Actual State' ls 7, \
     prefix.'state.txt' every ::index::index using 1:2:(5.0*cos($3)):(5.0*sin($3)) notitle with vectors as 8

unset output

if (index < steps) reread
