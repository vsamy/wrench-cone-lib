
# Overview

This library implements the Contact Wrench Cone as given [here](https://scaron.info/papers/journal/caron-tro-2016.pdf). It uses cdd for the polyhedron computation and Eigen for the matrix part. Python bindings are also available.

The license that applies to the whole package content is **GPLv3.0**. Please look at the license.txt file at the root of this repository.

## Installation

First install [eigen-cddlib](https://github.com/vsamy/eigen-cddlib).

Then, install ``wrench-cone-lib`` following the standard CMake procedure:

```sh
git clone --recursive https://github.com/vsamy/wrench-cone-lib
cd wrench-cone-lib
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFX=<your_path>
make -j4
make install
```

### Optional compilation definitions

There is only one parameter you can apply that will change the output of the library.
This parameter is `PLUCKER_NOTATION` and modify the outputs of the 6d vector by interchanging the 3 sub-vectors of angular and translational parameters.
The plucker notation outputs the results so that `output = { ang_x, ang_y, ang_z, tra_x, tra_y, tra_z }`.
By default `output = { tra_x, tra_y, tra_z, ang_x, ang_y, ang_z }`.
To activate the plucker notation, append to the cmake `-DPLUCKER_NOTATION=ON`.

### About python version

You may need to install the adequate numpy python modules on your system, depending on the version of python you intend to use. On Debian like systems like ubuntu:

```bash
sudo apt-get update; sudo apt-get install python-pip python3-pip
sudo pip install numpy; sudo pip3 install numpy
```

## Testing

```sh
cd wrench-cone-lib/build
make test
```

## Examples

To build the example please append to cmake `-DBUILD_EXAMPLES=ON`.
