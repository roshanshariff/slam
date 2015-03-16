Installation
============

You will need the following dependencies: cmake, boost, eigen3

You also need to install g2o from https://github.com/RainerKuemmerle/g2o

G2O Install Instructions
------------------------

Install the following dependencies: cmake, suitesparse, eigen3, lapack, blas.
With MacPorts:
    sudo port install cmake eigen3 SuiteSparse

The following instructions will install to G2O_ROOT:
````
export G2O_ROOT="$HOME/.local/opt/g2o"
mkdir -p "$G2O_ROOT"

git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
cmake "-DCMAKE_INSTALL_PREFIX=$G2O_ROOT" ../
make
make install
````

SLAM Install Instructions
-------------------------

Install the following dependencies: boost, gnuplot (apart from cmake and eigen3)
With MacPorts:
    sudo port install boost gnuplot

````
git clone https://github.com/roshanshariff/slam.git
mkdir slam/build
cd slam/build
cmake ../src # G2O_ROOT should be set
make
````

To run (for example):
```
cd ..
./build/slam --dataset Plaza1 --mcmc-slam --mcmc-steps 10 --mcmc-end-steps 1000 --sensor-range-stddev 1 --slam-plot --slam-plot-isometry
```
