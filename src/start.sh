#!/bin/bash
#exec ./run/mpu &
#P1=$!
exec ./test/pid 0.3 0 0.12 0.13 0 0.12 0 -23  -j4
#P2=$!
#wait $P1 $P2
