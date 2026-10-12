#!/usr/bin/env bash
# Re-runs every thesis simulation with ns-3 and copies the CSV output into this folder
#
# Usage:   NS3_DIR=~/ns-3.42 ./run-simulations.sh            (4-path runs, a few minutes)
#          NS3_DIR=~/ns-3.42 ./run-simulations.sh --with-256 (adds the 256-path runs, much longer)
set -euo pipefail

NS3_DIR="${NS3_DIR:?Set NS3_DIR to your ns-3 folder, e.g. NS3_DIR=~/ns-3.42}"
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
WITH_256="${1:-}"

cp "$REPO_DIR"/*.cc "$NS3_DIR/scratch/"
cd "$NS3_DIR"
mkdir -p results
./ns3 build

run() {   # run <program> <mode flags...>
  local prog="$1"; shift
  echo ">>> $prog $*"
  ./ns3 run "scratch/$prog $*"
}

for sim in spine-leaf-sim fat-tree-sim; do
  run "$sim"                       # Packet Spraying (baseline)
  run "$sim" --ptecmp=true         # PT-ECMP
  run "$sim" --flowlet=true        # Flowlet Switching
done

run spine-leaf-advanced
run spine-leaf-advanced --ptecmp=true
run spine-leaf-advanced --flowlet=true
run spine-leaf-advanced --letflow=true

if [ "$WITH_256" = "--with-256" ]; then
  for sim in spine-leaf-256path fat-tree-256path; do
    run "$sim"
    run "$sim" --ptecmp=true
    run "$sim" --flowlet=true
  done
fi

for f in results/*.csv; do          # ns-3 writes underscores; this repo uses hyphens
  cp "$f" "$REPO_DIR/$(basename "$f" | tr '_' '-')"
done
echo "Done: CSV files copied to $REPO_DIR"
