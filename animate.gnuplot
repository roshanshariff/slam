index = index + 1

set output prefix.index.'.gif'

plot prefix.'landmarks.txt' using 1:2 title 'Landmarks' with points ls 1, \
     prefix.'landmarks.estimated.'.index.'.txt' using 1:2 title 'Estimated Landmarks' with points ls 2, \
     prefix.'state.expected.txt' every ::::index using 1:2 title 'Measured State' with lines ls 3, \
     prefix.'state.expected.txt' every ::index::index using 1:2:(5.0*cos($3)):(5.0*sin($3)) notitle with vectors ls 4, \
     prefix.'state.estimated.'.index.'.txt' using 1:2 title 'Estimated State' with lines ls 5, \
     prefix.'state.estimated.'.index.'.txt' every ::index::index using 1:2:(5.0*cos($3)):(5.0*sin($3)) notitle with vectors ls 6, \
     prefix.'state.txt' every ::::index using 1:2 title 'Actual State' with lines ls 7, \
     prefix.'state.txt' every ::index::index using 1:2:(5.0*cos($3)):(5.0*sin($3)) notitle with vectors ls 8

unset output

if (index < steps) reread
