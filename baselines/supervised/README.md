## GNN baseline (supervised)

This directory contains all the code to train a ML baseline: a 2-layer Graph Neural Network (GNN) architecture with GraphConv convolutions. The model is trained using supervised learning, where training data is generated using the oracle solver.
Hyperparameter tuning has not been performed extensively, therefore much performance can be gained by a proper hyperparameter optimization scheme.
You may build on this, for example by optimizing the hyperparameters and experimenting with different types of convolutions and neural architectures.

### Install requirements
This baseline requires [PyTorch](https://pytorch.org/) and [PyTorch Geometric](https://pytorch-geometric.readthedocs.io/). Install using:
```
# If you want to use GPU, do not use pip but [install PyTorch](https://pytorch.org/get-started/locally/) manually and run below commands instead (without #)
# pip install torch-scatter torch-sparse -f https://data.pyg.org/whl/torch-$(python -c "import torch; print(torch.__version__)").html
# pip install torch-geometric
# Run from root of repository (install CPU version using pip)
pip install -r baselines/supervised/requirements.txt
```

### Generate data, train and test a model
Quick example to generate 10 data points using two instances with 5 seeds each, use it to train a model and test the model within the controller (using short time limits and few epochs for a quick example):
```
# Run from root of repository
python baselines/supervised/generate_training_data.py  --instance instances/ORTEC-VRPTW-ASYM-0bdff870-d1-n458-k35.txt --epoch_tlim 2 --oracle_tlim 5 --verbose
python baselines/supervised/generate_training_data.py  --instance instances/ORTEC-VRPTW-ASYM-08d8e660-d1-n460-k42.txt --epoch_tlim 2 --oracle_tlim 5 --verbose
python baselines/supervised/process_training_data.py
python baselines/supervised/train.py --num_epochs 10
python controller.py --instance instances/ORTEC-VRPTW-ASYM-0bdff870-d1-n458-k35.txt --epoch_tlim 5 -- python solver.py --verbose --strategy supervised --model_path baselines/supervised/ckpt
```

### Generate training data
In order to generate training data, you need to first execute the script `python baselines/supervised/generate_training_data.py` from the root directory. This script requires the name
of the instance which will be used to generate data from. The command line interface is equivalent to that of `solver.py`. Therefore, participants may make
use of the utility script `run_parallel_solver.sh` to generate training data for all instances as well.

Running `generate_training_data.py` will create a directory `data`, which is supposed to collect all the data from all the instances. After that, you can
run `python baselines/supervised/process_training_data.py`, which aggregates and transform the data to be ready to be used by the `train.py` script in this directory. The output of this
script is one file for the *input features* `(data.X.pkl)` and one file for the *ground truth labels* `(data.Y.pkl)`.

### Download training data
Ready to use training data can be downloaded [here](https://drive.google.com/file/d/1pLUzCviGsOdgv5wHA2oiLitX_NaXLcZ2/view?usp=sharing). Make sure to put the files in the folder `baselines/supervised/data`.

### Train the model
```
  python baselines/supervised/train.py
```
This script will dump model's checkpoints to the directory `ckpt` located in the root directory.

### Inference
To use the trained model as a strategy, you just need to pass `--strategy supervised` to either `solver.py` or `controller.py` along with the checkpoint
path with `--model_path <CKPT_DIR>`.

### Pretrained model
We provide a pretrained GNN baseline to ease reproducibility of the performance. The checkpoint is saved at `baselines/supervised/pretrained` and can be tested
by passing `--strategy supervised --model_path ./baselines/supervised/pretrained` to `solver.py`.

The current model takes as input a complete graph representing the current VRP epoch instance. Node features comprise:
- A binary feature indicating whether the node is the depot.
- Normalized node demands.
- A binary feature indicating whether the node must be dispatched in current epoch.
The cost of driving along the edge direction is used as the only edge feature.

Participants are free to enrich node and edge features with anything valuable to ease learning and performance of the neural network.

### Test and create submission
Make sure your solver can run through `baselines/supervised/run.sh`, then use the `baselines/supervised/create_submission.sh` script to create a submission  (you can omit any training code but include the trained model and necessary files!).
```
python controller.py --instance instances/ORTEC-VRPTW-ASYM-0bdff870-d1-n458-k35.txt --epoch_tlim 5 -- baselines/supervised/run.sh
baselines/supervised/create_submission.sh
# Test the submission to see if all dependencies where packed correctly
cd submissions/tmp
./install.sh && python ../../controller.py --instance ../../instances/ORTEC-VRPTW-ASYM-0bdff870-d1-n458-k35.txt --epoch_tlim 5 -- ./run.sh
cd ../..
```

#### Hyperparameters & Performances
| Hyperparameter | Value |
|----------------|-------|
| Learning Rate  | 0.01  |
| Weight Decay   | 0.005 |
| Hidden Neurons | 64    |

| Train | Validation | Test |
|-------|------------|------|
| ~66%  | ~68%       | ~68% |
