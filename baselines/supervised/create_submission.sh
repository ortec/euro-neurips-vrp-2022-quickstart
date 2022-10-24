#!/bin/bash
DATE=`date "+%Y-%m-%d-%H-%M-%S"`
mkdir -p submissions
cd submissions
rm -rf tmp
mkdir -p tmp/baselines/hgs_vrptw/{src,include}
cp -r ../baselines/hgs_vrptw/CMakeLists.txt tmp/baselines/hgs_vrptw
cp -r ../baselines/hgs_vrptw/src/{*.cpp,CMakeLists.txt} tmp/baselines/hgs_vrptw/src
cp -r ../baselines/hgs_vrptw/include/*.h tmp/baselines/hgs_vrptw/include
cp -r ../baselines/hgs_vrptw/extern tmp/baselines/hgs_vrptw
cp -r ../baselines/strategies tmp/baselines
# Extra for supervised baseline
mkdir -p tmp/baselines/supervised
cp -r ../baselines/supervised/{net.py,utils.py,transform.py} tmp/baselines/supervised
cp -r ../baselines/supervised/pretrained tmp/baselines/supervised/pretrained
# Use requirements and run.sh from supervised baseline
cp ../baselines/supervised/{run.sh,requirements.txt} tmp
cp ../{solver.py,tools.py,environment.py,install.sh,metadata} tmp
cd tmp
zip -o -r --exclude=*.git* --exclude=*__pycache__* --exclude=*.DS_Store* ../submission_$DATE.zip .;
cd ../..
echo "Created submissions/submission_$DATE.zip"