#!/bin/bash

prefix=$1
runs=100
cores=2

eval echo {1..$runs} | xargs -n 1 -P $cores -I XXX ./simulator -o "${prefix}runXXX."

summary_files="echo \\\"\\\${prefix}run{1..$runs}.summary.txt\\\""
for i in "Elapsed time" "Trajectory error" "Landmark error"; do
    echo -n "$i: "
    eval sed -n "'s/$i: \(.*\)/\1/p'" $(eval $summary_files) | average
done
