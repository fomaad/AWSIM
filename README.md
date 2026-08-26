# AWSIM

This is a fork and extended version of [Autoware Foundation's AWSIM](https://autowarefoundation.github.io/AWSIM/), supporting some advanced behaviors for NPC vehicles and pedestrians.

## Additional Features

- Dynamic actions to control NPC behaviors, such as, lane change, acceleration and deceleration profiles.
- Two scenario description languages to specify desired scenarios: [AWSIM-Script](https://github.com/fomaad/AWSIM-ScriptPy/blob/main/Origin-AWSIM-Script.md) and [AWSIM-ScriptPy](https://github.com/fomaad/AWSIM-ScriptPy).
The usage of both languages is explained in the [AWSIM-ScriptPy repo](https://github.com/fomaad/AWSIM-ScriptPy), so please check it out for more details.

## Installation

The environment requirements are listed here: https://autowarefoundation.github.io/AWSIM/GettingStarted/QuickStartDemo/#demo-contents.

Please follow the installation instructions in the Autoware repo: https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/#tune-dds-settings to make:
- DDS configuration
- CycloneDDS configuration
- NVIDIA GPU driver installation (Skip if already installed).

## Launching Binary Release

You can download a binary release from [here](https://github.com/fomaad/AWSIM/releases/download/v1.0.0/AWSIM.zip), unzip it, and launch the simulator using:

```bash
./awsim.x86_64
```

It may take some time for the application to start.

By default, Gaussian noise is added to the simulated data of LiDAR sensors. Use option `-noise` false to disable this noise.

```bash
./awsim.x86_64 -noise false
```

## Using AWSIM-Script and AW-Runtime-Monitor
AW-Runtime-Monitor (https://github.com/duongtd23/AW-Runtime-Monitor) is a runtime monitor that:
- Records traffic participants' dynamics and ADS (Autoware) internal state (e.g., planning trajectories, control commands, perceived objects, etc.) during simulation and dumps the information to a trace file once the simulation finishes.
- Can monitor the safety of a control command produced by ADS, and if it is unsafe, activate AEB.

The idea of using AWSIM-Script and AW-Runtime-Monitor together with AWSIM and Autoware is shown in the figure below. 
We can replace  AWSIM-Script with other scenario description language, e.g., Scenic (interested user can check the [extended Scenic](https://github.com/fomaad/Scenic) to be work with AWSIM).

<img src="tool-chain.png" alt="Tool architecture" width="400"/>

To launch these tools together,
in addition to AWSIM (this repo), clone AWSIMScriptPy and AW-Runtime-Monitor repos.
#### 1. Launch AWSIM and Autoware.
For AWSIM, download and run the binary version as explained above.

For Autoware, a detailed installation and launch instruction is available in this repo: https://github.com/dtanony/Autoware0412.
Please follow the instructions until you can launch Autoware and connect it to AWSIM.


#### 2. Launch AW-Runtime-Monitor
Instructions to install and launch AW-Runtime-Monitor are available in its [repository](https://github.com/duongtd23/AW-Runtime-Monitor).

After launching Autoware and AWSIM and they are connected, run the following command in another terminal:
```bash
python main.py -o <path-to-folder-to-save-traces> -v false
```

where the options `-v false` disable shielding. By default, it is enabled.
Note that you need to source Autoware's setup file before launching the monitor.
For more details about the tool usage, use `python main.py -h`.

```bash
$ python main.py -h
usage: main.py [-h] [-o OUTPUT] [-f {json,yaml}] [-n NO_SIM] [-v {true,false}]

Runtime Monitor for Autoware and AWSIM simulator. Adjust the component to record data by modifying
file config.yaml

options:
  -h, --help            show this help message and exit
  -o OUTPUT, --output OUTPUT
                        Output trace file name (default: auto-generated with timestamp)
  -f {json,yaml}, --format {json,yaml}
                        either json or yaml (default: json)
  -n NO_SIM, --no_sim NO_SIM
                        Simulation number, use as suffix to the file name (default: 1)
```

#### 4. Run scenario with AWSIM-Script client library:
Instructions to specify and run a scenario with the original AWSIM-Script and AWSIM-ScriptPy (Python interface) are available at: https://github.com/fomaad/AWSIM-ScriptPy.

## Unity Project Setup
We recommend using the binary release for most use cases. However, if you want to run the simulator inside the Unity Editor or make some modifications, follow this instruction from AWSIM document to import this project to the Unity Editor: https://autowarefoundation.github.io/AWSIM/DeveloperGuide/SetupUnityProject/.
