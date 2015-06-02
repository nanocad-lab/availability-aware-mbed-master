#!/bin/bash
#
# Author: Mark Gottscho <mgottscho@ucla.edu>
#
# This script iterates through each git repository in the current directory and runs "git pull origin master" on each. To avoid username/password authentication bottleneck SSH key pairs may be set up.

echo "Pulling git updates for origin/master in each subdir..."
start_dir=$PWD
for dir in $(find . -type d -maxdepth 1)
do
	if [[ "$dir" == "." || "$dir" == ".." ]]; then
		continue;
	fi

	if [[ !(-d "$dir/.git") ]]; then
		continue;
	fi

	echo "Next: $dir"
	cd $dir
	git pull origin master
	cd $start_dir
done

echo "DONE"
