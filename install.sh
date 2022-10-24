#!/bin/bash
pip install -r requirements.txt
cd baselines/hgs_vrptw
cmake . -DCMAKE_BUILD_TYPE=Release
cmake --build .
cd ../..