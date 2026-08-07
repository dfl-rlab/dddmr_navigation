#!/bin/bash
use_jproc=2
memory_size=$(grep MemTotal /proc/meminfo | tr -s ' ' | cut -d ' ' -f2)
if [ "$memory_size" -gt 8000000 ]; then
  echo "Memory is more than 8 GB, use 8 thread to compile"
  cd /tmp/gtsam && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_USE_SYSTEM_METIS=ON -DPCL_ENABLE_MARCHNATIVE=OFF .. && make install -j$(nproc)
else
  echo "Memory is less than 8 GB, use 4 thread to compile"
  cd /tmp/gtsam && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_USE_SYSTEM_METIS=ON -DPCL_ENABLE_MARCHNATIVE=OFF .. && make install -j4
fi

