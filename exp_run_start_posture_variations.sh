#!/bin/sh
for i in {0..2079}
do
    python3 -O exp_undisturbed_walking.py -s "$i"
done