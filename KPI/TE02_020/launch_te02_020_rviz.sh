#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rviz2 -d "${SCRIPT_DIR}/te02_020_collision_avoidance.rviz"
