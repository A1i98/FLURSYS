#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
if (($# == 0)) || [[ $1 == --help || $1 == -h ]]; then
    cat <<'USAGE'
Usage: scripts/flamegraph.sh <cargo flamegraph options> [-- <program arguments>]
Example: scripts/flamegraph.sh --no-default-features --bin flursys -- project solve /path/to/workspace --threads 1 --iterations 200
Install: cargo install flamegraph --locked; Linux also requires perf.
Output: target/profiling/flamegraph.svg and, on Linux, target/profiling/perf.data.
Override PERF, FLURSYS_FLAMEGRAPH_OUTPUT, or FLURSYS_PERF_COMMAND if needed.
USAGE
    if (($# == 0)); then exit 2; fi
    exit 0
fi
if ! command -v cargo-flamegraph >/dev/null 2>&1; then
    printf '%s\n' 'Install cargo-flamegraph: cargo install flamegraph --locked' >&2
    exit 1
fi
mkdir -p target/profiling
output=${FLURSYS_FLAMEGRAPH_OUTPUT:-target/profiling/flamegraph.svg}
if [[ $(uname -s) == Linux ]]; then
    local_perf_root=${XDG_DATA_HOME:-"$HOME/.local/share"}/flursys/profiling
    if [[ -z ${PERF:-} ]] && ! command -v perf >/dev/null 2>&1 && [[ -x "$local_perf_root/usr/bin/perf" ]]; then
        export PERF="$local_perf_root/usr/bin/perf"
        export LD_LIBRARY_PATH="$local_perf_root/usr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
    if ! command -v "${PERF:-perf}" >/dev/null 2>&1; then
        printf '%s\n' 'Install perf or set PERF to its executable path.' >&2
        exit 1
    fi
    export RUSTFLAGS="${RUSTFLAGS:+$RUSTFLAGS }-Clink-arg=-Wl,--no-rosegment"
    record=${FLURSYS_PERF_COMMAND:-record -e cpu-clock:u -F 199 --user-callchains --call-graph dwarf,16384 -o target/profiling/perf.data}
    exec cargo flamegraph --package flursys --profile profiling --no-inline --output "$output" --cmd "$record" "$@"
fi
exec cargo flamegraph --package flursys --profile profiling --no-inline --output "$output" "$@"
