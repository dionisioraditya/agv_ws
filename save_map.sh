#!/usr/bin/env bash
# Wrapper shortcut untuk menyimpan peta ROS 2 Nav2 & membuat paket .zip Map Studio
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/save_and_package_map.sh" "$@"
