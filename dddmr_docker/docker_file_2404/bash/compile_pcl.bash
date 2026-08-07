#!/bin/bash
memory_size=$(grep MemTotal /proc/meminfo | tr -s ' ' | cut -d ' ' -f2)
if [ "$memory_size" -gt 8000000 ]; then
  echo "Memory is more than 8 GB, use 8 thread to compile"
  cd /tmp/pcl && mkdir build && cd build && cmake -DPCL_ENABLE_AVX=OFF -DPCL_ENABLE_SSE=OFF -DPCL_ENABLE_MARCHNATIVE=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr .. && make install -j$(nproc)
else
  echo "Memory is less than 8 GB, use 4 thread to compile"
  cd /tmp/pcl && mkdir build && cd build && cmake -DPCL_ENABLE_AVX=OFF -DPCL_ENABLE_SSE=OFF -DPCL_ENABLE_MARCHNATIVE=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr .. && make install -j4
fi

