#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 1 ]]; then
    printf 'Usage: start_board.sh /absolute/path/config.json\n' >&2
    exit 2
fi
config_path=$(realpath -- "$1")
host_path=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
deployment_root=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["root"])' "$config_path")
export PYTHONPATH="$deployment_root/python:$host_path${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$deployment_root/bin${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec /usr/bin/python3 "$host_path/ai_coordinator/main.py" --config "$config_path" serve
