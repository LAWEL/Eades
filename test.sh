#!/bin/bash
if [ $# -ne 1 ]; then
    echo "usage: ./test.sh output_file" 1>&2
    exit 1
elif
    make
    ./eades
    neato -n1 -Tpng $1 -O
    open $1.png
fi
