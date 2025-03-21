# cybership_common

## Installation

Clone the repositor under ROS workspace
```bash
cd $ROS_WORKSPACE/src
git clone https://github.com/NTNU-MCS/cybership_software_suite
```

Get all the submodules
```bash
cd $ROS_WORKSPACE/src/cybership_software_suite
git submodule update --init --recursive
```

Build a virtual environment
```bash
cd $ROS_WORKSPACE
python3 -m venv venv --system-site-packages --symlinks
source venv/bin/activate
touch venv/COLCON_IGNORE
```

Install python dependencies to virtual environment
```bash
cd $ROS_WORKSPACE
source venv/bin/activate
find src/cybership_software_suite -name "requirements*txt" -exec pip install -r {} \;
```

Install dependencies
```bash
cd $ROS_WORKSPACE
rosdep install --from-paths src -i
```

Build the workspace
```bash
cd $ROS_WORKSPACE
colcon build --symlink-install
```

Source the workspace
```bash
source $ROS_WORKSPACE/venv/bin/activate
source $ROS_WORKSPACE/install/setup.bash
```

## Usage

### Simulation
To launch the simulation, run the following command:

```bash
ros2 launch cybership_simulator simulator.launch.py vessel_name:=voyager vessel_model:=voyager
```

### Physical

To launch the physical, run the following command:
```bash
ros2 launch cybership_bringup voyager.launch.py vessel_name:=voyager vessel_model:=voyager
```

## Docker

```bash
docker compose --profile voyager up
```