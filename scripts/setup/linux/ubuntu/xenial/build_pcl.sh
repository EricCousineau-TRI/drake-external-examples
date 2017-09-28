#!/bin/bash
set -e -u

# Pending resolution of build conflicts with existing Ubuntu-/ ROS-packaged PCL,
# this builds PCL 1.7.2. for use with Drake by overlaying with Drake's use of
# Eigen.
#
# Before running this, please ensure you have run `install_prereqs`.

DRAKE_REV=master
PCL_REV=pcl-1.7.2

export-prepend() {
    eval "export $1=\"$2:\$$1\""
}

env-extend() {
    local python_version=2.7;
    local prefix=${1%/};
    export-prepend PYTHONPATH $prefix/lib:$prefix/lib/python${python_version}/dist-packages:$prefix/lib/python${python_version}/site-packages;
    export-prepend PATH $prefix/bin;
    export-prepend LD_LIBRARY_PATH $prefix/lib;
    export-prepend PKG_CONFIG_PATH $prefix/lib/pkgconfig:$prefix/share/pkgconfig;
    echo "[ FHS Environment extended: ${prefix} ]"
}


tmp_dir=$(mktemp -d)
install_dir=${tmp_dir}/install
drake_install_prefix=${install_dir}/drake
pcl_install_prefix=${install_dir}/pcl

cd ${tmp_dir}
mkdir src

# Clone `drake` from master, build, and install.
(
    cd src
    git clone http://github.com/RobotLocomotion/drake -b ${DRAKE_REV}
    cd drake

    bazel run //:install -- ${install_prefix}/drake
)

# Use drake's install FHS, and build / install PCL in a separate directory.
(
    env-extend ${drake_install_prefix}

    cd src
    git clone http://github.com/pointcloudlibrary/pcl.git -b ${PCL_REV}
    cd pcl

    mkdir build && cd build
    eigen_args="
      -DEIGEN_INCLUDE_DIR:PATH=${drake_install_prefix}/include/eigen3
      -DEIGEN_INCLUDE_DIRS:PATH=${drake_install_prefix}/include/eigen3
      -DEIGEN3_INCLUDE_DIR:PATH=${drake_install_prefix}/include/eigen3
      "

    cmake .. \
        -DCMAKE_INSTALL_PREFIX=${pcl_install_prefix} \
        -DCMAKE_CXX_FLAGS="-std=c++11 -fext-numeric-literals" \
        ${eigen_args} \
        -DWITH_OPENNI=OFF -DWITH_OPENNI2=OFF -DWITH_LIBUSB=OFF \
        -DWITH_VTK=OFF -DWITH_QT=OFF \
        -DBUILD_TESTS=OFF -DBUILD_global_tests=OFF -DBUILD_examples=OFF
    make -j8 install

    # Strip rpath information.
    cd ${pcl_install_prefix}
    find bin lib | xargs chrpath -d
)

(
    # TODO(eric.cousineau): Build test program as a means to ensure this works.
    env-extend ${drake_install_prefix}
    env-extend ${pcl_install_prefix}
)

# Now package the install root
# ...
