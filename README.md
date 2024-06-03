# swarm-energy-replenishment

[**Installation**](#installation) | [**Usage**](#usage) | [**License**](#license)

This repository contains the code for the paper submission to ANTS 2024:
- A comparative study of energy replenishment strategies for robot swarms

# Installation

The following steps have been tested in Ubuntu 22, but it should be applicable to other Ubuntu distributions as well.

You need to have [ARGoS](https://www.argos-sim.info/) installed on your computer before proceeding.

Install the following apt packages:

```bash
sudo apt update
sudo apt install python3 python3-venv pip git libyaml-cpp-dev nlohmann-json3-dev
```

### Install plugins

Install the [plugins](https://gitlab.com/genki_miyauchi/multi-human-swarm-control-plugins) used in this project:

```bash
git clone https://gitlab.com/genki_miyauchi/multi-human-swarm-control-plugins.git
cd multi-human-swarm-control-plugins
mkdir build 
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr ../src
make
sudo make install
```

After executing the above commands, you should see ```e-puck_charger``` and ```rectangle_task``` appear when using ```argos3 -q entities```

Once the plugins have been installed, you may delete the ```minimal-length-swarm-networks-plugins``` folder

### Install protobuf

Protobuf is used to log the experiment in binary format. Install protobuf using the following commands.

```bash
wget https://github.com/protocolbuffers/protobuf/releases/download/v21.12/protobuf-cpp-3.21.12.tar.gz
tar xzf protobuf-cpp-3.21.12.tar.gz
cd protobuf-3.21.12/
./configure
make -j$(nproc)
sudo make install
sudo ldconfig
```
Once the protobuf compiler is successfully installed, you should be able to run ```protoc``` in the terminal.

### Python dependencies

Install the Python dependencies in requirements.txt using your choice of virtual environment. Here, we assume using venv:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

# Usage

### Build the project

```bash
cd swarm-energy-replenishment
mkdir results
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr ../src
make
```

### Run the project

Use one of the following commands to run an experiment:

```bash
argos3 -c experiments/work_and_charge/fixed_charger.argos
argos3 -c experiments/work_and_charge/mobile_charger.argos
```

For running batch simulation trials, configure and execute ```run_simulations.sh```.

# License
The code in this repository is released under the terms of the MIT license.
