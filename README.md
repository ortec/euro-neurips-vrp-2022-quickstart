
[![CI_Build](https://github.com/ortec/euro-neurips-vrp-2022-quickstart-dev/actions/workflows/CI.yml/badge.svg)](https://github.com/ortec/euro-neurips-vrp-2022-quickstart-dev/actions/workflows/CI.yml)

# Quickstart for EURO Meets NeurIPS 2022 Vehicle Routing Competition
Quickstart code for the [EURO Meets NeurIPS 2022 Vehicle Routing Competition](https://euro-neurips-vrp-2022.challenges.ortec.com/).

# Introduction
The EURO Meets NeurIPS 2022 Vehicle Routing Competition focuses on the classic Vehicle Routing Problem with Time Windows (VRPTW), as well as a dynamic variant in which orders arrive at different epochs during the day. Important: all submitted solvers compete *on both problem variants*, which is facilitated using the provided baseline strategy to use a static solver to solve the dynamic variant.
The complete description of the problem setting is provided on the [main webpage of the competition](https://euro-neurips-vrp-2022.challenges.ortec.com/).

This repository provides all the necessary code to start the competition. It includes a simple baseline method based on HGS-VRPTW as a static solver, along with examples of the use of the controller code designed to evaluate the algorithms.

# Stay updated!
Note: we will keep updating this repository with additional baselines, tools, information about code submission etc. to help you get most out of this competition! To stay updated, check back regularly, [follow us on Twitter](https://twitter.com/EuroNeuripsVRP) and join the [Slack workspace](https://join.slack.com/t/euro-neurips-vrp-2022/shared_invite/zt-1bfifn8ye-~azqWWXts1cR0YVURZNBmw), which is also the place to ask questions! Don't forget to [register your team](https://euro-neurips-vrp-2022.challenges.ortec.com/#registration)!

# Installation

The evaluation scripts are provided in Python, and the baseline solver (HGS-VRPTW) is implemented in C++.
We recommend to create a virtual environment using `Python 3.8+` to run the codes. 
Therefore, make sure that Python is installed, along with venv and a C++ compiler and make. On Windows, we recommend using [Visual Studio](https://visualstudio.microsoft.com/) or using [MinGW](https://sourceforge.net/projects/mingw/) and installing make through [Chocolatey](https://chocolatey.org/install) (run `choco install make` as administrator).
Then, run the following commands (Linux or Mac OS):

```shell
virtualenv -p python3.8 env
source env/bin/activate
./install.sh
```

Once this installation and compilation of the C++ code is done, you can directly run the baseline `solver.py` within the `controller.py` script for the dynamic variant of an instance:

```shell
python controller.py --instance instances/ORTEC-VRPTW-ASYM-0bdff870-d1-n458-k35.txt --epoch_tlim 5 -- ./run.sh
```

To solve the static variant of a problem instance, add `--static`:

```shell
python controller.py --instance instances/ORTEC-VRPTW-ASYM-0bdff870-d1-n458-k35.txt --epoch_tlim 5 --static -- ./run.sh
```

# Baseline solver: Hybrid Genetic Search (HGS)

The baseline solver uses internally the state-of-the-art Hybrid Genetic Search (HGS) algorithm by Vidal et al., which produces state-of-the-art results for over fifty variants of vehicle routing problems. The HGS creates new solutions by recombination and local search. Detailed information on the underlying algorithm mechanisms can be found in the papers listed below in this section. In particular, [2] recently provided a simple implementation of this algorithm (HGS-CVRP) with an additional local-search neighborhood called SWAP*. This code is distributed in [open-source](https://github.com/vidalt/HGS-CVRP) in C++, with additional wrappers in Julia and Python. Support for delivery time-windows has been documented in [3], and many other problem variants have been solved in [4] with extensions of the same algorithm.

An extension of HGS-CVRP [5] has been recently used to win the [VRPTW track](http://dimacs.rutgers.edu/programs/challenge/vrp/vrptw/) of the [DIMACS implementation challenge](http://dimacs.rutgers.edu/programs/challenge/vrp/). 
It uses the time-window evaluation strategies as in [3], along with an additional [SREX crossover](https://link.springer.com/content/pdf/10.1007/978-3-642-15844-5_54.pdf), dynamic parameters for population and neighborhood size, and additional code optimizations over the original [HGS-CVRP](https://github.com/vidalt/HGS-CVRP) implementation. This optimized implementation is used as a baseline in this competition and included in the `baselines/hgs_vrptw` folder.

It is important to note that all the vehicle routing variants covered with HGS until now were static, i.e., all information was known at the start of the solution process. The EURO Meets NeurIPS 2022 Vehicle Routing Competition goes beyond this scope, as the delivery requests are revealed dynamically in the dynamic problem variant. Extending a static solver to deal with dynamic requests is a significant challenge. The baseline request dispatch strategies provided in this quickstart provide initial starting strategies to do this extension, but many additional strategies and improvements are likely possible.

The implemented baseline strategies are *greedy*, *lazy* and *random*. Greedy will dispatch all requests in each epoch, while lazy will dispatch only must-go requests. Random will dispatch the must-go requests, and will dispatch each of the other requests with 50% probability (independently). Additionally, we provide an *oracle* 'baseline' that solves the problem in hindsight, i.e. it uses future information (by resetting the environment), to solve the problem offline in hindsight, using HGS with an extra constraint to respect the release times for requests. This oracle is NOT a valid strategy, as the solver should not reset the environment, but it can be used e.g. for imitation learning. Note that the oracle is not always better than the other baselines, as the hindsight problem is quite large and difficult to optimize, while the HGS algorithm was not specifically designed for this purpose.

* [1] [Vidal et al., Operations Research, 60(3), 611-624, (2012)](https://www.cirrelt.ca/DocumentsTravail/CIRRELT-2011-05.pdf) 
* [2] [Vidal, Computers & Operations Research, 140, 105643, (2022)](https://arxiv.org/pdf/2012.10384.pdf) 
* [3] [Vidal et al., Computers & Operations Research, 40(1), 475-489 (2013)](https://www.cirrelt.ca/DocumentsTravail/CIRRELT-2011-61.pdf) 
* [4] [Vidal et al., European Journal of Operational Research, 234(3), 658-673 (2014)](https://www.cirrelt.ca/DocumentsTravail/CIRRELT-2013-22.pdf) 
* [5] [Kool et al., DIMACS Competition Technical Report (2022)](https://wouterkool.github.io/pdf/paper-kool-hgs-vrptw.pdf)

# Files in this repository

## Scripts
* `controller.py` is an example controller script that can be used to evaluate your submission. It defines the protocol, therefore THIS FILE SHOULD NOT BE CHANGED.
* `solver.py` contains a baseline implementation for the solver, which can be used as a starting point for your own solver. It implements several strategies to repeatedly use HGS-VRPTW (in `baselines/hgs_vrptw/genvrp`) to solve the static VRPTW problem for each epoch.
* `run.sh` is the script that should contain the command to run the actual solver, which will be used for every instance on the code submission platform.
* `install.sh` (optional) is the script that installs the solver, which will be used once on the code submission platform.
* `run_parallel_solver.sh` is a simple script that can be used to run the solver for multiple instances in parallel.

## Other files
* `metadata` is a file that needs to be included when submitting your solver on the CodaLab submission platform (see below)
* `requirements.txt` specifies the Python requirements for your solver, make sure necessary dependencies are included and installed through the install.sh script.
* `environment.py` implements the dynamic VRPTW environment and a class that can use stdin/stdout to interact with the environment via `controller.py`
* `tools.py` contains tools for reading/writing static VRPTW instance files and validating solutions

# How to participate in the competition?
After [registering your team](https://euro-neurips-vrp-2022.challenges.ortec.com/#registration), we recommend to read the [detailed rules document](https://euro-neurips-vrp-2022.challenges.ortec.com/#rules) carefully and inspect the quickstart code, baselines and environment. Then, we suggest you implement your solver in `solver.py` such that it can use the existing code for interacting with `controller.py`, which will be used to evaluate your solver. Even if your solver is not (completely) written in Python, we still recommend wrapping it using Python to facilitate interaction with the environment, as is done in the provided baseline which wraps the HGS C++ algorithm (this is no strict requirement, see below).

# Evaluating your solver
For final evaluation for the dynamic variant of the problem, your solver will be called using the `controller.py` script:

    python controller.py --instance {instance_filename} --instance_seed {instance_seed} --epoch_tlim {epoch_tlim} -- ./run.sh

For the static variant of the problem, the call will be:

    python controller.py --instance {instance_filename} --static --epoch_tlim {epoch_tlim} -- ./run.sh
The `run.sh` script should contain the command to run your solver for the dynamic problem, following the protocol in `solver.py`, which provides a working example (we use a time limit of 5 seconds per epoch for the purpose of the example). For quick testing of different variants, you can directly provide the solver command instead of `./run.sh`, but your final solver should run through `run.sh`:

    python controller.py --instance instances/ORTEC-VRPTW-ASYM-0bdff870-d1-n458-k35.txt --epoch_tlim 5 -- python solver.py --verbose --strategy random

For convenience during development, the `solver.py` script implements the arguments `--instance`, `--instance_seed`, `--static` and `--epoch_tlim` that can be used to directly run an instance using the environment, without running `controller.py`:

    python solver.py --instance instances/ORTEC-VRPTW-ASYM-0bdff870-d1-n458-k35.txt --verbose

Note that it is *not* required to use `solver.py`, or use Python at all, as long as the solver implements the same protocol to interact via stdin/stdout (the exact protocal can be inferred from the code by logging the messages). Also note that *both* static and dynamic instances will be evaluated using the same protocol: for static instances there will simply be just a single epoch.

IMPORTANT: validate that your solver runs well within the `controller.py` script before submission. This means that the solver can *only* use the information returned by the environment when calling `.reset()` or `.step()` (including the second info return variable with additional info), and any other methods or properties should not be used!

# Submitting your solver

## 1. Register your team on the CodaLab platform
READ THESE INSTUCTIONS CAREFULLY. Every team must register their team on CodaLab:
* Go to https://codalab.lisn.upsaclay.fr/competitions/6627
* Create an account on CodaLab specifically for your team. Every team should use just ONE account to make submissions!
* On the top right, click on your account name, go to 'Settings', and *enter your team name* under 'Competition Settings'.
* Note: Teams with multiple accounts or missing or unregistered team names will be removed from the competition!

## 2. Create a submission zip file

To submit your solver, you must package everything that is needed in a zip file and submit it using the [CodaLab Competition](https://codalab.lisn.upsaclay.fr/competitions/6627) platform. The following files are required in the root of the zip file:
* `metadata`: this file should be include to indicate a code submission. There is no need to change it.
* `install.sh`: (optional) script to compile your solver and/or install required dependencies *through pip*. This will be executed once for each submission.
* `run.sh`: the entrypoint script for your solver. It should NOT use any arguments.
* All other files needed to run your solver.

Make sure that everything your solver needs is included or installed through `install.sh`. For convenience, we provide the `create_submission.sh` script that can do this for you: it creates a `tmp` folder in the folder `submissions` with only the necessary files, for which it will create a zip. Make sure to add your dependencies to the script if you use it.

Note that you can include `environment.py`, `tools.py` etc. in your submission, so these can be used inside your solver, and you are free to modify these files as well. You can even include a `controller.py` script, but it will be ignored. The code submission platform will use its own controller and environment to evaluate your submission.

## 3. Submit your solver on CodaLab
* On the CodaLab platform, navigate to [Participate -> Submit / View Results](https://codalab.lisn.upsaclay.fr/competitions/6627#participate-submit_results).
* In the top, select the *test* phase for your first submission (NOT the qualification phase)
* Upload your zip file by pressing the Submit button. Once you press the button, a window will pop up to select your file. After selecting the file, it will be directly submitted.
* Wait for your submission to be processed. You can refresh the status by clicking 'refresh'. Your submission was succesful if you see 'Finished' as status.
* If your submission fails, you can use the output/error files to troubleshout your submission. Most likely, errors will be in the 'ingestion error log' (the ingestion step runs your submission).
* If there are any problems, you may also try to run your submission locally inside the Singularity/Apptainer computational environment (see below).
* Note that processing time of your submission may vary depending on the availability of the compute workers and submission load. If your submission does not show status 'running' within a few hours, it may help to submit an extra test submission to trigger a compute worker.
* Contact us on [Slack](https://join.slack.com/t/euro-neurips-vrp-2022/shared_invite/zt-1bfifn8ye-~azqWWXts1cR0YVURZNBmw) if you are unable to solve your problem.

The competition has three 'phases':
* The *test* 'phase' is not really a phase (it overlaps the other phases) but can be used to test if your solver can be evaluated correctly by the CodaLab platform. It will install your solver and test it using a single static and a dynamic instance with a short time limit. The standard output/error of the install script, and the controller running your solver can be inspected to troubleshout problems. 
* The *qualification* phase is were you should submit your final solver, AFTER it has succesfully been evaluated in the test phase. You can make a submission *at most once per day* (therefore, first test your submission using the test phase). Your best submission will be visible on the leaderboard. Your last submission of this phase will be used for final evaluation, if you are within the top 10 on the leaderboard on October 31st. Important: while the leaderboard shows your BEST submission, your LAST submission will be used in the final phase. Make sure to submit your final solution before the deadline (October 31st).
* The *final (hidden)* phase is hidden. The top 10 solutions from the qualification phase will be automatically evaluated during this phase. It is NOT possible to provide a new submission.

For more details about the phases (time limits etc.), read the [rules](https://euro-neurips-vrp-2022.challenges.ortec.com/#rules) document.

Important: the code submission platform will compute two ranks for all solvers: one based on the average performance (minimizing total route driving duration) on the static variant, and one based on the average performance on the dynamic variant. It is NOT possible to submit a solver for just one of the variants. The overall rank will be determined by the average of the rank for the static variant and the dynamic variant (with the overal average performance as tie-braker), so you should make sure your solver performs well for both problem variants.

## Computational environment using Apptainer/Singularity
The code will be executed inside an [Apptainer](https://apptainer.org/)/Singularity container, which is based on [this Dockerfile](https://github.com/TCatshoek/codalab-dockers/blob/master/legacy-gpu/Dockerfile). See the Dockerfile for pre-installed dependencies. The environment has a single CPU core ([Intel(R) Xeon(R) Gold 5118 CPU @ 2.30GHz](https://ark.intel.com/content/www/us/en/ark/products/120473/intel-xeon-gold-5118-processor-16-5m-cache-2-30-ghz.html)), a single GPU ([Nvidia TitanRTX 24GB](https://ark.intel.com/content/www/us/en/ark/products/120473/intel-xeon-gold-5118-processor-16-5m-cache-2-30-ghz.html)) and 32 GB of RAM. (Note that the 2.3GHz CPU may be slower than your local machine!)

### Testing locally with Apptainer/Singularity
To test your submission locally in the same environment, you can install [Apptainer](https://apptainer.org/) on Ubuntu (Windows/Mac see below):
* Download a release from https://github.com/apptainer/apptainer/releases (e.g. apptainer_1.0.3_amd64.deb)
* Run `sudo dpkg -i apptainer_1.0.3_amd64.deb && sudo apt install -f`
* Run `singularity shell docker://b0xcat/codalab-legacy:gpu` to obtain an interactive shell (add `--nv` flag if your system has a GPU which you want to use inside the container). The first time this command may take a while.
* Run `./install.sh` and `python controller.py --instance instances/ORTEC-VRPTW-ASYM-0bdff870-d1-n458-k35.txt --epoch_tlim 5 -- ./run.sh` and verify everything runs correctly

For Windows/Mac you need an Ubuntu virtual machine (VM), see [here](https://apptainer.org/docs/admin/main/installation.html#installation-on-windows-or-mac). Inside the VM you can use the steps above (which are simpler than the official installation instructions).

### Use of commercial software
Note: it is your responsibility to ensure appropriate licenses such that your solver can run on the code submission platform. If you want to use a commercial MIP solver, we recommend using [Gurobi](https://www.gurobi.com/), which has agreed to provide licenses for the code submission platform (this is NOT available yet). If you require a Gurobi license for developing your solver (and you cannot use an [academic license](https://www.gurobi.com/academia/academic-program-and-licenses/)), contact us or [request an evaluation license](https://www.gurobi.com/downloads/request-an-evaluation-license/). Contact us on [Slack](https://join.slack.com/t/euro-neurips-vrp-2022/shared_invite/zt-1bfifn8ye-~azqWWXts1cR0YVURZNBmw) if you have questions about using commercial software.

# Possible ideas
To encourage participation in the competition, we provide some suggestions on various ways to participate in this competition using machine learning techniques.
* Improve the Hybrid Genetic Search algorithm used in `solver.py`:
    * Use machine learning to select which parents to crossover in the genetic algorithm to generate offspring
    * Use machine learning based initial constructions
    * Train a policy to manage the size of the population, neighbourhood and other important hyperparameters (see the [paper](https://wouterkool.github.io/pdf/paper-kool-hgs-vrptw.pdf))
    * Use machine learning to define intelligent neighbourhoods/heatmaps as a better 'nearest neighbour' definition in the local search, see e.g. [this paper](https://arxiv.org/abs/2102.11756) or [this paper](https://arxiv.org/pdf/1912.11462.pdf)
    * Use the learning to search principle to improve the local search
    * ...
* Learn a good dynamic strategy to select which orders to dispatch:
    * Using sample-efficient reinforcement learning (as the environment is slow), for example using replay buffers
    * Using supervised learning based on 'hindsight-optimal' example solutions
    * ...
* Learn end-to-end approaches to solve the dynamic problem:
    * For example based on [the attention model](https://arxiv.org/abs/1803.08475)
    * ...
