#!/bin/bash
strategy=$1  # [greedy, lazy, random]
nw=$2  # parallelism degree

instances=( $(ls -1 ./instances/*) )
num_instances=${#instances[@]}

for i in $(seq 0 $nw $num_instances)
do
    for j in $(seq 0 $nw)
    do
        let "idx = $i + $j"
        if [ $idx -lt $num_instances ] ;
           then
               python solver.py --instance ${instances[$idx]} --strategy $strategy &
        fi
    done
    wait
done
