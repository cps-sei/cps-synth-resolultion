# Toolchain Installation Instructions for macOS
- **Author**: Simon Chu, edited by Ben Gafford
- **Reference**: Benjamin Gafford's `README-REU2019.md`
- **Date**: Tue Jul 28 16:50:22 EDT 2020 (Summer 2020)
- **Operating System**: macOS Catalina 10.15.6
- **Java Version**: 1.8.0_251 

## What does this instruction include? (alternative instructions to README.md)
1. **jMAVSim** (branch: multidrone) [**Original**](https://github.com/gamoreno/jMAVSim/tree/multidrone) [**Backup**](https://github.com/sychoo/jMAVSim/tree/multidrone)
2. **PX4 Firmware** (branch: 9bfc4f2d540b3592b3c519d3bbf1e607e7374238) [**Original**](https://github.com/PX4/Firmware) [**Backup**](https://github.com/sychoo/Firmware)
3. **MAVLink Router** (branch: osx) [**Original**](https://github.com/gamoreno/mavlink-router/tree/osx) [**Backup**](https://github.com/sychoo/mavlink-router/tree/osx)
4. **MissionApp** (branch: multidrone-multithread) [**Original, Requires Authentication**](https://bitbucket.org/gamoreno/missionapp/src/multidrone-multithread/)

## 1. Install jMAVSim
What is jMAVSim?
- A simple multirotor simulator with MAVLink protocol support
- It will support the flight simulator (GUI) for the Mission App

Prerequisites (Software Requirement):
- [Homebrew](https://brew.sh/)
- [Apache Ant](https://ant.apache.org/)
  - Install Apache Ant using the following command
    ```bash
      brew install ant
    ```

**Step 1**: Clone the repository (branch: multidrone) and initialize the submodule
```bash
git clone --branch multidrone https://github.com/gamoreno/jMAVSim.git

git submodule init
git submodule update
```

**Step 2**: Build the project, and create a standalone JAR (Java Archive) file
```bash
cd jMAVSim/
ant create_run_jar copy_res
cd out/production
java -Djava.ext.dirs= -jar jmavsim_run.jar
```

Note that when you want to recompile the project, please delete the `out/` directory as follows and rerun the build command above
```bash
cd jMAVSim
rm -rf out/
```

For more information, please refer to the [**README.md**](https://github.com/gamoreno/jMAVSim/blob/multidrone/README.md) file for the [**jMAVSim**](https://github.com/gamoreno/jMAVSim/tree/multidrone) repository

## 2. Install PX4 Firmware
What is PX4 Firmware?
- PX4 is a open-source drone software with flight control, simulation and testing capability. It can be use for consumer drones and industrial applications. Firmware consists of jMAVSim multi-rotor flight simulator and a variety of flight control and simulation software.
- It will be used to create multiple instances of drones, make sure 2 drones are shown in jMAVSim for Mission App.

Prerequisites (Software Requirement):
- [Homebrew](https://brew.sh/)

**Step 1**: Increase the maximum number of open files
```bash
ulimit -S -n 300
```

**Step 2**: Install common PX4 development tools on Homebrew
```bash
brew tap PX4/px4
brew install px4-dev
brew install px4-sim
```

**Step 3**: Install XQuartz and Java
```bash
brew cask install xquartz java
```

**Step 4**: Install required Python packages using pip (package installer for Python)
```bash
sudo easy_install pip
sudo -H pip install pyserial empy toml numpy pandas jinja2 pyyaml
```

**Step 5**: Download PX4 Firmware source code from GitHub, and checkout the branch that works seamlessly with the Mission App
```bash
git clone https://github.com/PX4/Firmware.git
cd Firmware
git checkout 9bfc4f2d540b3592b3c519d3bbf1e607e7374238
```
Of course, you are always encouraged to explore the new version of the PX4 Firmware. As of the time of the creation of this documentation, the latest version is `v1.10.2`. You can view and checkout the branch as follows
```bash
# list all releases tags
git tag -l 

# checkout v1.10.2
git checkout v1.10.2
```

Step 6: Building the source code
```bash
make px4_sitl jmavsim
```

The following message indicates that the build succeeded
```bash
Build files have been written to: /home/youruser/src/Firmware/build/px4_fmu-v4_default
[954/954] Creating /home/youruser/src/Firmware/build/px4_fmu-v4_default/px4_fmu-v4_default.px4
```

Note that when you want to recompile the project, please delete the `out/` directory as follows and rerun the build command above
```bash
cd Firmware
make clean
```

For more information, please refer to the [**PX4 Toolchain Installation Documentation**](https://dev.px4.io/v1.10/en/setup/dev_env_mac.html) file for the [**PX4 Firmware**](https://github.com/PX4/Firmware) repository

## 3. Install MAVLink Router
What is MAVLink Router?
- A software that routes MAVLink packets between endpoints in the MAVLink framework.
- It will forward the port from the PX4 firmware flight controller to both the `ego drone` and the `follower drone`. 

**Step 1**: Download the repository (branch: osx)
```bash
git clone --branch osx https://github.com/gamoreno/mavlink-router.git
```

**Step 2**: Update the submodule for the repository
```bash
cd mavlink-router/
git submodule update --init --recursive
```
**Step 3**: Build and install the software
```bash
./autogen.sh && ./configure CFLAGS='-g -O2' \
        --sysconfdir=/etc --localstatedir=/var --libdir=/usr/lib64 \
    --prefix=/usr --disable-systemd

make
sudo make install
```

Note that `--disable-systemd` option needs to be added to macOS build because of macOS does not support `systemd`

Note that a configuration file with [**a sample format**](https://raw.githubusercontent.com/gamoreno/mavlink-router/osx/examples/config.sample) for MAVLink can be added, but it is not necessary for Mission App

For more information, please refer to the [**README.md**](https://github.com/gamoreno/mavlink-router/blob/osx/README.md) file for the [**MAVLink Router**](https://github.com/gamoreno/mavlink-router/tree/osx) repository, or the [**port information chart**](./charts/port_information.png) created by Gabriel.

## 4. Run the Mission App
What is Mission App
- Mission App uses DroneSDK to interact with jMAVSim flight simulator, and to make the drones perform different tasks

**Step 1**: Install DroneSDK (branch: enforcer)
```bash
git clone --branch enforcer https://github.com/gamoreno/DronecodeSDK.git
cd DronecodeSDK
git submodule update --init --recursive

make default
sudo make default install
```

**Step 2**: (Required for the first time): After installing DroneSDK for the first time, run the following command
```bash
sudo ldconfig
```

**Step 3**: Compile C++ source code (missionapp.cpp, follower.cpp)
```bash
make
cd follower
make

# go to root directory for the software installation
cd ../../ 
```

**Step 4**: Make sure all installed PX4 software are in the same directory (PX4 Firmware, jMAVSim, MAVLink Router, and Mission App)
```console
$ ls
Firmware       jMAVSim        mavlink-router missionapp
```

**Step 5**: Start the execution bash script `run_multidrone.sh`/setup bash script `setup_multidrone.sh`.
```bash
cd missionapp
./run_multidrone.sh
```

Refer to [**README.md**](README.md) for more information. 

## Appendix 1: Check and change Java version (macOS)
Check the current-running Java Runtime version
```console
$ java -version

java version "1.8.0_251"
Java(TM) SE Runtime Environment (build 1.8.0_251-b08)
Java HotSpot(TM) 64-Bit Server VM (build 25.251-b08, mixed mode)
```

Check all the installed Java version
```console
$ /usr/libexec/java_home -V

Matching Java Virtual Machines (9):
    14.0.1, x86_64:	"Java SE 14.0.1"	/Library/Java/JavaVirtualMachines/jdk-14.0.1.jdk/Contents/Home
    13.0.2, x86_64:	"Java SE 13.0.2"	/Library/Java/JavaVirtualMachines/jdk-13.0.2.jdk/Contents/Home
    12.0.1, x86_64:	"Java SE 12.0.1"	/Library/Java/JavaVirtualMachines/jdk-12.0.1.jdk/Contents/Home
    11.0.7, x86_64:	"Java SE 11.0.7"	/Library/Java/JavaVirtualMachines/jdk-11.0.7.jdk/Contents/Home
    10.0.2, x86_64:	"Java SE 10.0.2"	/Library/Java/JavaVirtualMachines/jdk-10.0.2.jdk/Contents/Home
    9.0.4, x86_64:	"Java SE 9.0.4"	/Library/Java/JavaVirtualMachines/jdk-9.0.4.jdk/Contents/Home
    1.8.0_251, x86_64:	"Java SE 8"	/Library/Java/JavaVirtualMachines/jdk1.8.0_251.jdk/Contents/Home
    1.8.0_65, x86_64:	"Java SE 8"	/Library/Java/JavaVirtualMachines/jdk1.8.0_65.jdk/Contents/Home
    1.7.0_79, x86_64:	"Java SE 7"	/Library/Java/JavaVirtualMachines/jdk1.7.0_79.jdk/Contents/Home

/Library/Java/JavaVirtualMachines/jdk-14.0.1.jdk/Contents/Home
```

Select/change the current running Java Runtime (put the following statement to the corresponding `.bashrc`/`.zshrc` file)
```bash
export JAVA_HOME=`/usr/libexec/java_home -v 1.8.0_251, x86_64`
```
