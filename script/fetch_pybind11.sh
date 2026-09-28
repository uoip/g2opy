#!/usr/bin/env bash
#
# EXTERNAL/pybind11 is not tracked in this repository (see .gitignore), so a
# fresh clone has to fetch it before cmake can configure.  This installs the
# version python/CMakeLists.txt expects into EXTERNAL/pybind11.
#
set -euo pipefail

VERSION=2.13.6
SHA256=e08cb87f4773da97fa7b5f035de8763abc656d87d5773e62f6da0587d1f0ec20
URL="https://github.com/pybind/pybind11/archive/refs/tags/v${VERSION}.tar.gz"

dest="$(cd "$(dirname "$0")/.." && pwd)/EXTERNAL/pybind11"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

echo "downloading pybind11 ${VERSION} ..."
curl -fsSL -o "$tmp/pybind11.tar.gz" "$URL"
echo "${SHA256}  $tmp/pybind11.tar.gz" | sha256sum -c -

tar xzf "$tmp/pybind11.tar.gz" -C "$tmp"
src="$tmp/pybind11-${VERSION}"

# Install only what the build needs, to keep the tree small: the headers, the
# CMake glue, and pybind11's own CMakeLists.txt (python/CMakeLists.txt pulls it
# in with ADD_SUBDIRECTORY to get the pybind11::* targets defined).
rm -rf "$dest"
mkdir -p "$dest"
cp "$src/CMakeLists.txt" "$src/LICENSE" "$src/README.rst" "$dest/"
cp -r "$src/include" "$src/tools" "$dest/"

echo "pybind11 ${VERSION} -> $dest"
