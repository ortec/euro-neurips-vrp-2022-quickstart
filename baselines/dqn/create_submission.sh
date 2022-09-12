#!/bin/bash
DATE=`date "+%Y-%m-%d-%H-%M-%S"`
mkdir -p submissions
cd submissions
rm -rf tmp
mkdir -p tmp/baselines/hgs_vrptw
cp -r ../baselines/hgs_vrptw/{*.cpp,*.h,Makefile} tmp/baselines/hgs_vrptw
cp -r ../baselines/strategies tmp/baselines
# Extra for dqn baseline
mkdir -p tmp/baselines/dqn
cp -r ../baselines/dqn/{net.py,utils.py} tmp/baselines/dqn
cp -r ../baselines/dqn/pretrained tmp/baselines/dqn/pretrained
# Use requirements and run.sh from dqn baseline
cp ../baselines/dqn/{run.sh,requirements.txt} tmp
cp ../{solver.py,tools.py,environment.py,install.sh,metadata} tmp
cd tmp
zip -o -r --exclude=*.git* --exclude=*__pycache__* --exclude=*.DS_Store* ../submission_$DATE.zip .;
cd ../..
echo "Created submissions/submission_$DATE.zip"