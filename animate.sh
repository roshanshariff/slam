#!/bin/bash

prefix=$1
steps=$(sed -n 's/Time steps: \(.*\)/\1/p' < "${prefix}summary.txt")

gnuplot <<EOF
prefix='$prefix'
steps=$steps

unset autoscale
set xrange [-150:100]
set yrange [-100:100]

set key top left

set style line 1 lc rgbcolor 'red' pt 6 ps 1.5
set style line 2 lc rgbcolor 'black' pt 3 ps 1
set style line 3 lc rgbcolor 'black' lw 5
set style line 4 lc rgbcolor 'green' lw 3

index=0

set terminal gif
load 'animate.gnuplot'
unset output

EOF

ffmpeg -i "${prefix}%d.gif" -vcodec libx264 -vpre normal "${prefix}video.mp4"
rm $(eval echo "${prefix}{1..${steps}}.gif")
rm $(eval echo "${prefix}landmarks.estimated.{1..${steps}}.txt")
rm $(eval echo "${prefix}trajectory.estimated.{1..${steps}}.txt")
