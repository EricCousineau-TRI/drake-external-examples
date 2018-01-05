#!/bin/bash
set -e -u

mkdir -p build && cd build
cmake .. -Ddrake_DIR=${DRAKE_INSTALL}/lib/cmake/drake -DVTK_DIR=${DRAKE_INSTALL}/lib/cmake/vtk-8.0
make simple_vtk_example
./src/vtk/simple_vtk_example

cd ../src/pcl_isolate
mkdir -p build && cd build
cmake .. -DSIMPLE_VTK_EXAMPLE_DIR=../../build/src/vtk/
make simple_pcl_example
./src/pcl/simple_pcl_example
