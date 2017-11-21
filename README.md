
Overview
=========

This library implements the Contact Wrench Cone as given [here](https://scaron.info/papers/journal/caron-tro-2016.pdf). It uses cdd for the polyhedron computation and Eigen for the matrix part. Python bindings are also available.

The license that applies to the whole package content is **CeCILL-C**. Please look at the license.txt file at the root of this repository.

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


Installation and Usage
=======================

The procedures for installing the wrench-cone-lib package and for using its components is based on the [PID](http://pid.lirmm.net/pid-framework/pages/install.html) build and deployment system called PID. Just follow and read the links to understand how to install, use and call its API and/or applications.

About authors
=====================

wrench-cone-lib has been developped by following authors: 
+ Vincent Samy (LIRMM)

Please contact Vincent Samy (vsamy@outlook.fr) - LIRMM for more information or questions.




