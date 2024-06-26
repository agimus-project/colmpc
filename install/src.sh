#!/bin/bash -eux

CMAKE_PREFIX_PATH=${1:-$PWD/install}
MAKE_JOBS=${2:-2}

export CMAKE_PREFIX_PATH

declare -a SOURCES=(
    'jrl-umi3218/jrl-cmakemodules/master'
    'stack-of-tasks/eigenpy/v3.4.0'
    'humanoid-path-planner/hpp-fcl/devel'
    'stack-of-tasks/pinocchio/v2.7.0'
    'gepetto/example-robot-data/v4.1.0'
    'loco-3d/crocoddyl/v2.0.2'
    'cmake-wheel/mim_solvers/v0.0.4.c0'
    'agimus-project/colmpc/main'
)

for source in "${SOURCES[@]}"
do
    IFS="/" read -r -a split <<< "${source}"
    ORG="${split[0]}"
    PRJ="${split[1]}"
    TAG="${split[2]}"

    git clone --branch "$TAG" --depth 1 "https://github.com/$ORG/$PRJ"
    cmake -B "$PRJ/build" -S "$PRJ" \
        -DBUILD_TESTING=OFF \
        -DBUILD_WITH_COLLISION_SUPPORT=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$CMAKE_PREFIX_PATH" "${ARG[@]}" \
        -DCMAKE_INSTALL_LIBDIR=lib \
        -DHPP_FCL_HAS_QHULL=ON \
        -Wno-dev
    cmake --build "$PRJ/build" -j "$MAKE_JOBS"
    cmake --build "$PRJ/build" -t install
done
