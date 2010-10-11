#!/bin/bash

prefix=$1
runs=100
cores=2

eval echo {1..$runs} | xargs -n 1 -P $cores -I XXX ./simulator -o "${prefix}runXXX."

summary_files=$(eval echo \\\"\\\${prefix}run{1..$runs}.summary.txt\\\")
echo $summary_files
for i in "Elapsed time" "Landmark error" "State error"; do
    echo -n "$i: "
    eval sed -n "'s/$i: \(.*\)/\1/p'" $summary_files | average
done
