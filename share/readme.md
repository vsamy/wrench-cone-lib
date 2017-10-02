## Installation

There is some more struggle in order to use the python bindings.
Because python users are using numpy library to perform their computation, i wanted this lib to be able to handle numpy array as input.
To do so i used boost python library.
For now, PID does not provide proper handles for building boost python automatically, so here is the steps to do it cleanly.

Clone [pid-workspace](https://github.com/vsamy/pid-workspace) if you don't have one yet.

Then 
```bash
cd <pid-workspace>/pid
cmake ..
make deploy package=wrench-cone-lib
```

### Python users

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

```bash
cd <pid-workspace>/packages/wrench-cone-lib/build
git checkout integration
make build
```

Add the path to your PYTHONPATH and you are done.

```bash
export PYTHONPATH=$PYTHONPATH:<pid-workspace>/install/python2.7
```
