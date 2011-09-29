#!/bin/bash

prefix=$1

#steps=$(sed -n 's/Time steps: \(.*\)/\1/p' < "${prefix}summary.txt")
#seed=$(sed -n 's/Random seed: \(.*\)/\1/p' < "${prefix}summary.txt")
#iterations=$(sed -n 's/MCMC iterations: \(.*\)/\1/p' < "${prefix}summary.txt")

steps=$(wc -l < "${prefix}trajectory.txt")
steps=$((steps - 1))

mkdir "${prefix}video"

gnuplot <<EOF
prefix='$prefix'
steps=$steps

#set title 'MCMC-SLAM Simulation ($iterations iterations, seed: $seed)'
set title 'MCMC-SLAM Simulation'

unset autoscale
set xrange [-150:100]
set yrange [-100:100]

set key top left

set style line 1 lc rgbcolor 'red' pt 6 ps 1.5
set style line 2 lc rgbcolor 'black' pt 3 ps 1

set style line 3 lc rgbcolor 'gray' lw 2
set style arrow 4 head size 6,20 nofilled lc rgbcolor 'gray' lw 2

set style line 5 lc rgbcolor 'black' lw 5
set style arrow 6 head size 50,20,50 filled lt 5 lc rgbcolor 'black' lw 10

set style line 7 lc rgbcolor 'green' lw 3
set style arrow 8 head size 50,20,50 filled lt 7 lc rgbcolor 'green' lw 10

index=0

set terminal gif
load 'animate.gnuplot'
unset output

EOF

ffmpeg -y -v 1 -i "${prefix}video/%d.gif" -vcodec libx264 -vpre lossless_fast "${prefix}video.mp4"

rm $(eval echo "${prefix}video/{1..${steps}}.gif")
#rm $(eval echo "${prefix}landmarks.estimated.{1..${steps}}.txt")
#rm $(eval echo "${prefix}state.estimated.{1..${steps}}.txt")
