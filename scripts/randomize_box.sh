#!/usr/bin/env bash
# Run inside Docker after Gazebo is running and the box is released.
# Usage: bash scripts/randomize_box.sh [--count N] [--interval SECONDS]
#        [-- Python randomizer options...]
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
count=0
interval=10
while [[ $# -gt 0 ]]; do
  case "$1" in
    --count|--interval)
      if [[ $# -lt 2 ]]; then echo "Missing value for $1" >&2; exit 2; fi
      if [[ "$1" == --count ]]; then count="$2"; else interval="$2"; fi
      shift 2
      ;;
    --) shift; break ;;
    -h|--help)
      echo "Usage: bash scripts/randomize_box.sh [--count N] [--interval SECONDS] [-- Python options...]"
      echo "Defaults: repeat until Ctrl+C; wait 10 seconds between completed moves."
      exit 0
      ;;
    *) echo "Unknown option: $1 (use -- before Python options)" >&2; exit 2 ;;
  esac
done
if [[ ! "$count" =~ ^[0-9]+$ || ! "$interval" =~ ^[0-9]+$ ]]; then
  echo "Count and interval must be nonnegative integers." >&2
  exit 2
fi
count=$((10#$count))
interval=$((10#$interval))

if ! command -v ign >/dev/null 2>&1; then
  echo "Warning: ign not found. Saving sampled scenes only; Gazebo will not be moved." >&2
fi

trap 'echo "Stopped randomizing."; exit 130' INT
trap 'exit 143' TERM

run_id="$(date -u +%Y%m%dT%H%M%SZ)_$$_${RANDOM}"
index=1
echo "Automatically moving the box. Stop with Ctrl+C."
echo "Use only while the arm is idle: this loop does not wait for expert actions."
while (( count == 0 || index <= count )); do
  printf -v episode 'random_%s_%06d' "$run_id" "$index"
  seed=$(( (RANDOM << 15) | RANDOM ))
  echo "Episode: $episode | Seed: $seed"
  python3 "$script_dir/randomize_box.py" \
    "$@" --episode "$episode" --seed "$seed"
  index=$((index + 1))
  if (( count == 0 || index <= count )); then sleep "$interval"; fi
done
