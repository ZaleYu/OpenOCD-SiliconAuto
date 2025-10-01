#!/usr/bin/env bash
set -euo pipefail

# ==============================
# Config (adjust as needed)
# ==============================
# Installation output directories
PREFIX_LINUX="${HOME}/openocd-build/linux"
PREFIX_WIN64="${HOME}/openocd-build/win64"

# Cross compiler triplet prefix
HOST_WIN64="x86_64-w64-mingw32"

# (Optional) Windows dependency root directory
# (must contain libusb-1.0 / hidapi with .pc/.a/.dll.a files)
# Change this to your own prefix if you installed them manually
WIN64_DEPS_PREFIX="$HOME/Desktop/win64-libs/prefix"

# OpenOCD source directory
# (if this script is in "scripts/", then its parent should be the OpenOCD root)
# You can also set an absolute path, e.g.: SRC_DIR="$HOME/OpenOCD"
SRC_DIR="$(cd "$(dirname "$0")" && pwd)"

# Extra compiler flags (suppress some warnings, do not treat warnings as errors)
EXTRA_CFLAGS="-Wno-error -Wno-unused-parameter -Wno-missing-field-initializers"
EXTRA_CXXFLAGS="${EXTRA_CFLAGS}"

# Common configure options:
# - only enable jlink and cmsis-dap
# - use internal jimtcl/libjaylink
# - disable -Werror
COMMON_CFG_OPTS=(
  --disable-werror
  --enable-internal-jimtcl
  --enable-internal-libjaylink
  --enable-jlink
  --enable-cmsis-dap
)

# ==============================
# Functions
# ==============================
usage() {
  cat <<EOF
Usage:
  $0 linux        # Build Linux native version
  $0 win64        # Cross-compile Windows x86_64 version

Environment requirements:
  linux:  system gcc/g++/make, and libusb-1.0-dev, hidapi-dev (recommended)
  win64:  MinGW-w64 toolchain (${HOST_WIN64}-gcc etc.)
          Prebuilt Windows libusb-1.0 and hidapi with .pc files in WIN64_DEPS_PREFIX
EOF
}

ensure_configure() {
  # If configure does not exist (e.g. fresh git clone), run bootstrap first
  if [[ ! -x "${SRC_DIR}/configure" ]]; then
    echo "==> Running bootstrap (autogen) ..."
    (cd "${SRC_DIR}" && ./bootstrap)
  fi
}

build_linux() {
  echo "=== Building OpenOCD for Linux (native) ==="
  local builddir="${SRC_DIR}/build-linux"
  rm -rf "${builddir}"
  mkdir -p "${builddir}"
  pushd "${builddir}" >/dev/null

  PKG_CONFIG_PATH="${PKG_CONFIG_PATH:-}"
  export PKG_CONFIG_PATH

  CFLAGS="${EXTRA_CFLAGS}" \
  CXXFLAGS="${EXTRA_CXXFLAGS}" \
  "${SRC_DIR}/configure" \
    --prefix="${PREFIX_LINUX}" \
    "${COMMON_CFG_OPTS[@]}"

  make -j"$(nproc)"
  make install
  popd >/dev/null

  echo "=== Linux build DONE. Installed to: ${PREFIX_LINUX} ==="
}

build_win64() {
  echo "=== Building OpenOCD for Windows x86_64 (cross) ==="
  local builddir="${SRC_DIR}/build-win64"
  rm -rf "${builddir}"
  mkdir -p "${builddir}"
  pushd "${builddir}" >/dev/null

  # Point to Windows dependency libraries (libusb-1.0 / hidapi)
  # Must contain:
  #   ${WIN64_DEPS_PREFIX}/include
  #   ${WIN64_DEPS_PREFIX}/lib
  #   ${WIN64_DEPS_PREFIX}/lib/pkgconfig/{libusb-1.0.pc,hidapi.pc}
  export PKG_CONFIG_LIBDIR="${WIN64_DEPS_PREFIX}/lib/pkgconfig"
  export PKG_CONFIG_PATH="${PKG_CONFIG_LIBDIR}"

  # Cross compiler tools
  export CC="${HOST_WIN64}-gcc"
  export CXX="${HOST_WIN64}-g++"
  export AR="${HOST_WIN64}-ar"
  export RANLIB="${HOST_WIN64}-ranlib"
  export STRIP="${HOST_WIN64}-strip"

  # Ensure our prefix paths are searched first
  export CFLAGS="-I${WIN64_DEPS_PREFIX}/include ${EXTRA_CFLAGS}"
  export CXXFLAGS="-I${WIN64_DEPS_PREFIX}/include ${EXTRA_CXXFLAGS}"
  export LDFLAGS="-L${WIN64_DEPS_PREFIX}/lib"

  "${SRC_DIR}/configure" \
    --host="${HOST_WIN64}" \
    --prefix="${PREFIX_WIN64}" \
    "${COMMON_CFG_OPTS[@]}"

  make -j"$(nproc)"
  make install
  popd >/dev/null

  echo "=== Windows (win64) build DONE. Installed to: ${PREFIX_WIN64} ==="
  echo "Reminder: when copying to Windows, also include DLLs: libusb-1.0.dll, hidapi DLL, and possibly libstdc++/libgcc/libwinpthread."
}

# ==============================
# Entry
# ==============================
target="${1:-}"
if [[ -z "${target}" ]]; then
  usage; exit 1
fi

ensure_configure

case "${target}" in
  linux) build_linux ;;
  win64) build_win64 ;;
  *) usage; exit 1 ;;
esac

