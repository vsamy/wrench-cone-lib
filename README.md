
Overview
=========

This library implements the Contact Wrench Cone as given [here](https://scaron.info/papers/journal/caron-tro-2016.pdf). It uses cdd for the polyhedron computation and Eigen for the matrix part. Python bindings are also available.

The license that applies to the whole package content is **CeCILL-C**. Please look at the license.txt file at the root of this repository.

## Python users

There is some more struggle in order to use the python bindings.
Because python users are using numpy library to perform their computation, i wanted this lib to be able to handle numpy array as input.
To do so i used boost python library.
For now, PID does not provide proper handles for building boost python automatically, so here is the steps to do it cleanly.

*Before anything else*, build the package once (See section Installation and Usage). This will download all necessary libraries. Then get to the boost python installation section.

### Boost python installation

Download boost python version 1.64.0 [boost_1_64_0.tar.gz](http://www.boost.org/users/history/version_1_64_0.html)
then extract the files

```bash
cd <Donwload_folder>
tar xvzf boost_1_64_0.tar.gz
cd boost_1_64_0
```

Build and install the library in PID external packages

```bash
./bootstrap.sh --prefix=<PID-workspace>/external/<platform>/boost/1.64.0
./b2 -j4 install
```

Build the package one more time.
Add the path to your PYTHONPATH and you are done.

```bash
export PYTHONPATH=$PYTHONPATH:<pid-workspace>/install/python2.7
```


Installation and Usage
=======================

The procedures for installing the wrench-cone-lib package and for using its components is based on the [PID](https://gite.lirmm.fr/pid/pid-workspace/wikis/home) build and deployment system called PID. Just follow and read the links to understand how to install, use and call its API and/or applications.

About authors
=====================

wrench-cone-lib has been developped by following authors: 
+ vsamy (LIRMM)

Please contact vsamy (vincent.samy@lirmm.fr) - LIRMM for more information or questions.




