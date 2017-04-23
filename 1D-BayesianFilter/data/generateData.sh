#!/bin/bash

dir=$1
if [ ! $dir ]; then
    echo -e "\nPlease input directory name (e.g. example01)!"
    exit 1
fi

if [ ! -d $dir ]; then mkdir $dir; fi
if [ ! -d $dir/observations ]; then mkdir $dir/observations; fi

> ${dir}/control_data.txt
 
rm ${dir}/observations/observations_*
for ((i=0; i<14; i++))
do
    echo 1 >>${dir}/control_data.txt
    count=`echo $i | awk '{printf "%06i", $1+1}'`
    touch ${dir}/observations/observations_${count}.txt
done
