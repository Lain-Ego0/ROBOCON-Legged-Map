#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${GLIM_BUILD_DIR:-$ROOT/build/glim-runtime}"
CORE="$BUILD_DIR/libglim.so"
GPU_PLUGIN="$BUILD_DIR/libodometry_estimation_gpu.so"

for library in "$CORE" "$GPU_PLUGIN"; do
  if [[ ! -f "$library" ]]; then
    printf 'Missing build output: %s\n' "$library" >&2
    exit 2
  fi
done

python3 - "$CORE" "$GPU_PLUGIN" <<'PY'
import ctypes
import os
import sys

cuda = ctypes.CDLL("libcudart.so.13")
cuda.cudaGetDeviceCount.argtypes = [ctypes.POINTER(ctypes.c_int)]
cuda.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
cuda.cudaMemset.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_size_t]
cuda.cudaFree.argtypes = [ctypes.c_void_p]
count = ctypes.c_int()
result = cuda.cudaGetDeviceCount(ctypes.byref(count))
if result != 0 or count.value < 1:
    raise SystemExit(f"CUDA device discovery failed: rc={result}, count={count.value}")

buffer = ctypes.c_void_p()
result = cuda.cudaMalloc(ctypes.byref(buffer), 4096)
if result != 0:
    raise SystemExit(f"CUDA allocation failed: rc={result}")
result = cuda.cudaMemset(buffer, 0x5A, 4096)
if result != 0:
    cuda.cudaFree(buffer)
    raise SystemExit(f"CUDA memory write failed: rc={result}")
result = cuda.cudaFree(buffer)
if result != 0:
    raise SystemExit(f"CUDA deallocation failed: rc={result}")

ctypes.CDLL(os.path.abspath(sys.argv[1]), mode=os.RTLD_GLOBAL)
ctypes.CDLL(os.path.abspath(sys.argv[2]), mode=os.RTLD_NOW)
print(f"CUDA runtime passed on {count.value} device(s)")
print("GLIM core and GPU odometry plugin loaded successfully")
PY
