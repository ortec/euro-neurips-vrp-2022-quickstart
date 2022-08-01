#!/bin/bash
pip install -r requirements.txt
cd baselines/hgs_vrptw
make clean
make all
cd ../..