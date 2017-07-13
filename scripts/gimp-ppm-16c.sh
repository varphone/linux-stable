#!/bin/bash
#
# Format GIMP PPM to 16 columns aligned.
#
# The formated contents is output to stdout.
#
if [ -z $1 ]; then
	echo "Usage: $0 <ppm file>"
	exit 1
fi

N=0
C=0
while read line; do
	if ((N < 4)); then
		echo $line;
	else
		printf "%3s" $line
		C=$((C+1))
		if ((C < 16)); then
			printf " "
		else
			printf "\n"
			C=0
		fi
	fi
	N=$((N+1))
done < $1
