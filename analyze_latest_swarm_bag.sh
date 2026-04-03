#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT="$SCRIPT_DIR"

usage() {
  cat <<'EOF'
Usage:
  analyze_latest_swarm_bag.sh [bag_dir] [options]

If bag_dir is omitted, the newest rosbag directory under ./data is used.

Options:
  --data-root DIR      Search root for latest bag (default: ./data)
  --analysis-dir DIR   Output analysis directory (default: <bag_dir>/analysis)
  --profile MODE       Plot profile: full, paper, or both (default: both)
  --window MODE        Plot window: full, valid, or cmd_ctrl (default: full)
  --storage-id ID      rosbag2 storage backend (default: sqlite3)
  --skip-plots         Export CSV/summary only
  -h, --help           Show this help
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

require_arg() {
  [[ $# -ge 2 ]] || die "Missing value for $1"
}

find_latest_bag() {
  local search_root=$1
  local latest_dir=""
  local latest_mtime=-1
  local dir
  local mtime

  [[ -d "$search_root" ]] || die "Data root does not exist: $search_root"

  while IFS= read -r dir; do
    [[ -f "$dir/metadata.yaml" ]] || continue
    mtime=$(stat -c %Y "$dir")
    if (( mtime > latest_mtime )); then
      latest_mtime=$mtime
      latest_dir=$dir
    fi
  done < <(find "$search_root" -mindepth 1 -maxdepth 1 -type d)

  [[ -n "$latest_dir" ]] || die "No rosbag directories with metadata.yaml found under: $search_root"
  printf '%s\n' "$latest_dir"
}

run_python() {
  python3 "$@"
}

BAG_DIR=""
DATA_ROOT="$REPO_ROOT/data"
ANALYSIS_DIR=""
PROFILE="both"
WINDOW="full"
STORAGE_ID="sqlite3"
SKIP_PLOTS=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --data-root)
      require_arg "$@"
      DATA_ROOT=$2
      shift 2
      ;;
    --analysis-dir)
      require_arg "$@"
      ANALYSIS_DIR=$2
      shift 2
      ;;
    --profile)
      require_arg "$@"
      PROFILE=$2
      shift 2
      ;;
    --window)
      require_arg "$@"
      WINDOW=$2
      shift 2
      ;;
    --storage-id)
      require_arg "$@"
      STORAGE_ID=$2
      shift 2
      ;;
    --skip-plots)
      SKIP_PLOTS=1
      shift
      ;;
    -*)
      die "Unknown option: $1"
      ;;
    *)
      [[ -z "$BAG_DIR" ]] || die "Only one bag_dir positional argument is allowed"
      BAG_DIR=$1
      shift
      ;;
  esac
done

case "$PROFILE" in
  full|paper|both) ;;
  *) die "Invalid --profile: $PROFILE" ;;
esac

case "$WINDOW" in
  full|valid|cmd_ctrl) ;;
  *) die "Invalid --window: $WINDOW" ;;
esac

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # Make rosbag2_py and related ROS Python packages available.
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

if [[ -f "$REPO_ROOT/install/setup.bash" ]]; then
  # Prefer the local workspace environment when it exists.
  # shellcheck disable=SC1091
  set +u
  source "$REPO_ROOT/install/setup.bash"
  set -u
fi

if [[ -z "$BAG_DIR" ]]; then
  BAG_DIR=$(find_latest_bag "$DATA_ROOT")
fi

[[ -d "$BAG_DIR" ]] || die "Bag directory does not exist: $BAG_DIR"
[[ -f "$BAG_DIR/metadata.yaml" ]] || die "Not a rosbag directory: $BAG_DIR"

if [[ -z "$ANALYSIS_DIR" ]]; then
  ANALYSIS_DIR="$BAG_DIR/analysis"
fi

echo "Bag:         $BAG_DIR"
echo "Analysis:    $ANALYSIS_DIR"
echo "Profile:     $PROFILE"
echo "Plot window: $WINDOW"
echo "Storage ID:  $STORAGE_ID"
echo

echo "[1/2] Exporting rosbag to CSV"
run_python "$REPO_ROOT/fsmpx4/script/analyze_swarm_bag" \
  "$BAG_DIR" \
  --output-dir "$ANALYSIS_DIR" \
  --storage-id "$STORAGE_ID"

if (( SKIP_PLOTS )); then
  echo
  echo "CSV export complete: $ANALYSIS_DIR"
  exit 0
fi

echo
echo "[2/2] Generating plots"
case "$PROFILE" in
  full)
    run_python "$REPO_ROOT/generate_plots.py" "$ANALYSIS_DIR" --profile full --window "$WINDOW"
    ;;
  paper)
    run_python "$REPO_ROOT/generate_plots.py" "$ANALYSIS_DIR" --profile paper --window "$WINDOW"
    ;;
  both)
    run_python "$REPO_ROOT/generate_plots.py" "$ANALYSIS_DIR" --profile full --window "$WINDOW"
    echo
    run_python "$REPO_ROOT/generate_plots.py" "$ANALYSIS_DIR" --profile paper --window "$WINDOW"
    ;;
esac

echo
echo "Done."
echo "Bag:      $BAG_DIR"
echo "Analysis: $ANALYSIS_DIR"
