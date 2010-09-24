#!/bin/bash

simulator=$(dirname "$0")/simulator

runs=10
cores=2
runs_per_core=$(($runs / $cores))

core=0
while (($core < $cores)); do
    (
	mkdir $core
	cd $core
	i=$(($core * $runs_per_core))
	end=$(($i + $runs_per_core))
	while (($i < $end)); do
	    mkdir data
	    echo Run $(($i+1))
	    ../$simulator > /dev/null
	    sleep 2
	    mv data ../run_$((i+1))
	    i=$((i+1))
	done
	cd ..
    )&
    core=$(($core + 1))
done
wait
i=$(($core * $runs_per_core))
while (($i < $runs)); do
    mkdir data
    $simulator $i
    mv data run_$((i+1))
    i=$((i+1))
done
