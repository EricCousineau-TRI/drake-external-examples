#!/bin/bash
set -e -u

# This is a stopgap measure pending resolution of build conflicts with existing
# Ubuntu-/ ROS-packaged PCL.
# This builds PCL 1.7.2. for use with Drake by overlaying with Drake's use of
# Eigen.
# A more elegant method would be to pull in dependencies more in a more
# granular fashion, but this will be deferred to actually using system packages.
#
# Before running this, please ensure you have run `install_prereqs`.

PCL_VERSION=1.7.2

DRAKE_REV=master
PCL_REV=pcl-${PCL_VERSION}

export-prepend() {
    # usage: export-prepend <var> <value>
    # Prepends a value to a given environment variable.
    eval "export $1=\"$2:\${$1-}\""
}

env-extend() {
    # usage: env-extend <prefix>
    # Extends FHS to include the supplied directory.
    local python_version=2.7;
    local prefix=${1%/};
    export-prepend PYTHONPATH $prefix/lib:$prefix/lib/python${python_version}/dist-packages:$prefix/lib/python${python_version}/site-packages;
    export-prepend PATH $prefix/bin;
    export-prepend LD_LIBRARY_PATH $prefix/lib;
    export-prepend PKG_CONFIG_PATH \
        $prefix/lib/pkgconfig:$prefix/share/pkgconfig;
    echo "[ FHS Environment extended: ${prefix} ]"
}

tmp_dir=$(mktemp -d)
install_dir=${tmp_dir}/install
drake_install_prefix=${install_dir}/drake
pcl_install_prefix=${install_dir}/pcl

archive_path_format=${tmp_dir}/pcl-%s-drake-%s-xenial-x86_64.tar.gz

cd ${tmp_dir}
mkdir -p src

if false; then
# Clone `drake` from master, build, and install.
(
    cd src
    git clone http://github.com/RobotLocomotion/drake -b ${DRAKE_REV}
    cd drake

    bazel run //:install -- ${drake_install_prefix}
)

# Use drake's install FHS, and build / install PCL in a separate directory.
(
    env-extend ${drake_install_prefix}

    cd src
    git clone http://github.com/pointcloudlibrary/pcl.git -b ${PCL_REV}
    cd pcl

    mkdir -p build && cd build

    # TODO(eric.cousineau): 
    eigen_args="
      -DEIGEN_INCLUDE_DIR:PATH=${drake_install_prefix}/include/eigen3
      -DEIGEN_INCLUDE_DIRS:PATH=${drake_install_prefix}/include/eigen3
      -DEIGEN3_INCLUDE_DIR:PATH=${drake_install_prefix}/include/eigen3
      "

    # TODO(eric.cousineau): Re-enable -WITH_VTK=ON once the following errors
    # are resolved (by updating VTK packaging):
    # - `vtkVisibleCellSelector.h` not found.
    cmake .. \
        -DCMAKE_INSTALL_PREFIX=${pcl_install_prefix} \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS="-std=c++11 -fext-numeric-literals" \
        ${eigen_args} \
        -DWITH_OPENNI=OFF -DWITH_OPENNI2=OFF -DWITH_LIBUSB=OFF \
        -DWITH_VTK=OFF -DWITH_QT=ON \
        -DBUILD_TESTS=OFF -DBUILD_global_tests=OFF -DBUILD_examples=OFF
    make -j8 install

    # Strip rpath information.
    cd ${pcl_install_prefix}
    find bin lib -maxdepth 1 -type f | xargs chrpath -d
)
fi

(
    # Build test program as a means to ensure this fails fast if things are
    # amiss.
    env-extend ${drake_install_prefix}
    env-extend ${pcl_install_prefix}

    cd src
    git clone http://github.com/EricCousineau-TRI/repro.git pcl_build_test
    cd pcl_build_test/cpp/pcl

    mkdir -p build && cd build
    cmake ..
    make || { echo "Build failed."; exit 1; }
)

# Now package the install root
(
    drake_sha=$(cd src/drake && git rev-parse --short HEAD)
    archive_path=$(printf ${archive_path_format} ${PCL_VERSION} ${drake_sha})

    cd ${pcl_install_prefix}
    echo "[ Packaging: ${archive_path} ]"
    tar cfz ${archive_path} *
)
