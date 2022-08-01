#!/bin/bash
DATE=`date "+%Y-%m-%d-%H-%M-%S"`
mkdir -p submissions
cd submissions
rm -rf tmp
mkdir -p tmp
cp -r ../baselines tmp
rm tmp/baselines/hgs_vrptw/{*.o,genvrp}
cp ../{solver.py,tools.py,environment.py,run.sh,install.sh,requirements.txt,metadata} tmp
cd tmp
zip -o -r --exclude=*.git* --exclude=*__pycache__* --exclude=*.DS_Store* ../submission_$DATE.zip .;
cd ../..
echo "Created submissions/submission_$DATE.zip"