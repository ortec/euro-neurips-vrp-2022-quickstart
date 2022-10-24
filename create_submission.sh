#!/bin/bash
DATE=`date "+%Y-%m-%d-%H-%M-%S"`
mkdir -p submissions
cd submissions
rm -rf tmp
mkdir -p tmp/baselines/hgs_vrptw/{src,include,lib}
cp -r ../baselines/hgs_vrptw/{CMakeLists.txt,*.py} tmp/baselines/hgs_vrptw
cp -r ../baselines/hgs_vrptw/src/{*.cpp,CMakeLists.txt} tmp/baselines/hgs_vrptw/src
cp -r ../baselines/hgs_vrptw/lib/*.py tmp/baselines/hgs_vrptw/lib
cp -r ../baselines/hgs_vrptw/include/*.h tmp/baselines/hgs_vrptw/include
cp -r ../baselines/hgs_vrptw/extern tmp/baselines/hgs_vrptw
cp -r ../baselines/strategies tmp/baselines
cp ../{solver.py,tools.py,environment.py,run.sh,install.sh,requirements.txt,metadata} tmp
cd tmp
zip -o -r --exclude=*.git* --exclude=*__pycache__* --exclude=*.DS_Store* ../submission_$DATE.zip .;
cd ../..
echo "Created submissions/submission_$DATE.zip"