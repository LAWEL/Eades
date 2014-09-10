#!/bin/bash

# Documentation: Command-line Invocation
# http://www.graphviz.org/content/command-line-invocation

if [ $# -ne 1 ]; then
    echo "usage: ./test.sh output_file" 1>&2
    exit 1
else
    make &&
    ./eades $1 &&
    neato -n1 -Tpng $1 -O &&
    open $1.png
fi
