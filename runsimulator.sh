#!/bin/bash

prefix=$1
runs=100
cores=2

$simulator="Debug/slam"

date >> "${prefix}simulation_log.txt"
echo "Runs: $runs" >> "${prefix}simulation_log.txt"
echo "Cores: $cores" >> "${prefix}simulation_log.txt"

eval echo {1..$runs} | xargs -n 1 -P $cores -I XXX $simulator -o "${prefix}runXXX."

summary_files=$(eval echo \\\"\\\${prefix}run{1..$runs}.summary.txt\\\")
cut -d: -f1 "${prefix}run1.summary.txt" | while read name; do
    echo -n "$name: " >> "${prefix}simulation_log.txt"
    eval sed -n "'s/$name: \(.*\)/\1/p'" $summary_files \
	| average >> "${prefix}simulation_log.txt"
done
echo >> "${prefix}simulation_log.txt"
times >> "${prefix}simulation_log.txt"
echo >> "${prefix}simulation_log.txt"

for filename in landmarks{,.estimated} state{,.estimated,.expected} summary; do
    files=$(eval echo \\\"\\\${prefix}run{1..$runs}.$filename.txt\\\")
    eval rm $files
done
