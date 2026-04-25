# swarm-energy-replenishment

[**Installation**](#installation) | [**Usage**](#usage) | [**License**](#license)

This repository contains the code for the following papers:

- Optimal Performance of Cooperative Energy-Replenishment in Robot Swarms submitted to DARS 2026.
- [A Comparative Study of Energy Replenishment Strategies for Robot Swarms](https://doi.org/10.1007/978-3-031-70932-6_1) presented at ANTS2024 (tag: [ANTS2024](https://github.com/genkimiyauchi/swarm-energy-replenishment/tree/ANTS2024)).

## Installation

The following steps have been tested in Ubuntu 22, but it should be applicable to other Ubuntu distributions as well.

You need to have [ARGoS](https://www.argos-sim.info/) installed on your computer before proceeding.

Install the following apt packages:

```bash
sudo apt update
sudo apt install python3 python3-venv pip git libyaml-cpp-dev nlohmann-json3-dev
```

### Install plugins

Install the [plugins](https://gitlab.com/genki_miyauchi/argos-sct-plugins) used in this project:

```bash
git clone -b energy-aware https://gitlab.com/genki_miyauchi/argos-sct-plugins.git
cd argos-sct-plugins
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr ../src
make
sudo make install
```

After executing the above commands, you should see `e-puck_charger` and `rectangle_task` appear when using `argos3 -q entities`

Once the plugins have been installed, you may delete the `argos-sct-plugins` folder

### Install protobuf

Protobuf is used to log the experiment in binary format. Install protobuf using the following commands. This can take some time to install.

```bash
wget https://github.com/protocolbuffers/protobuf/releases/download/v21.12/protobuf-cpp-3.21.12.tar.gz
tar xzf protobuf-cpp-3.21.12.tar.gz
cd protobuf-3.21.12/
./configure
make -j$(nproc)
sudo make install
sudo ldconfig
```

Once the protobuf compiler is successfully installed, you should be able to run `protoc` in the terminal.

### Python dependencies

To use the plotting scripts in ```src/scripts```, install the Python dependencies in requirements.txt using your choice of virtual environment. Here, we assume using venv:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Usage

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

Use the following command to run an experiment:

```bash
./experiments/work_and_charge/run_simulation.sh
```

## License

The code in this repository is released under the terms of the MIT license.
