## Installation

Clone [pid-workspace](https://github.com/lirmm/pid-workspace.git) if you don't have it yet.

Then
```bash
cd <pid-workspace>/pid
cmake ..
make deploy package=wrench-cone-lib
```

### About python version

You may need to install the adequate numpy python modules on your system, depending on the version of python you intend to use. On Debian like systems like ubuntu:

```
sudo apt-get update; sudo apt-get install python-pip python3-pip
sudo pip install numpy; sudo pip3 install numpy
```

### Setting the PYTHONPATH

Add the path to your PYTHONPATH and you are done.

```bash
export PYTHONPATH=$PYTHONPATH:<pid-workspace>/install/python2.7
```

### Build example and docs
You can perform a `ccmake .` in the build directory of the git **integration** branch.
You can then mark **ON** the options for building the examples and doxygen files. Then `make build`
