#!/bin/bash
make
./eades
neato -n1 -Tpng test.dot -O
open test.dot.png
