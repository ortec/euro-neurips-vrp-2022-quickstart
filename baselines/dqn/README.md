## DQN baseline (reinforcement learning)

This directory contains all the code to train a ML baseline: a Deep Q-Network (DQN) consisting of a simple Multi-Layer Perceptron (MLP). The model is trained using reinforcement learning.
You may build on this, for example by optimizing the hyperparameters and experimenting with different types of neural architectures.

### Install requirements
This baseline requires [PyTorch](https://pytorch.org/) and [PyTorch Geometric](https://pytorch-geometric.readthedocs.io/). Install using:
```
# Run from root of repository
pip install -r baselines/supervised/requirements.txt
```

### Pretrained model
We provide a pretrained model. Note that it actually learned the greedy policy. To test the pretrained model:
```
python controller.py --instance instances/ORTEC-VRPTW-ASYM-0bdff870-d1-n458-k35.txt --epoch_tlim 5 -- python solver.py --verbose --strategy dqn --model_path baselines/dqn/pretrained
```

### Train and test a model
Example how to train a model and test the model within the controller:
```
# Run from root of repository
python baselines/dqn/train.py
python controller.py --instance instances/ORTEC-VRPTW-ASYM-0bdff870-d1-n458-k35.txt --epoch_tlim 5 -- python solver.py --verbose --strategy dqn --model_path baselines/dqn/ckpt
```

### Test and create submission
Make sure your solver can run through `baselines/dqn/run.sh`, then use the `baselines/dqn/create_submission.sh` script to create a submission (you can omit any training code but include the trained model and necessary files!).
```
python controller.py --instance instances/ORTEC-VRPTW-ASYM-0bdff870-d1-n458-k35.txt --epoch_tlim 5 -- baselines/dqn/run.sh
baselines/dqn/create_submission.sh
# Test the submission to see if all dependencies where packed correctly
cd submissions/tmp
./install.sh && python ../../controller.py --instance ../../instances/ORTEC-VRPTW-ASYM-0bdff870-d1-n458-k35.txt --epoch_tlim 5 -- ./run.sh
cd ../..
```