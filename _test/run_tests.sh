#!/usr/bin/env bash
set -u

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
failed=0

tests=(
    commands
    debug_sdr
    error_sdr
    math_sdr
    sensor
    telemetry
)

for test_name in "${tests[@]}"; do
    printf '\n=== Running %s tests ===\n' "$test_name"
    if ! make -C "$script_dir/mod/$test_name" test; then
        printf '*** %s tests failed ***\n' "$test_name" >&2
        failed=1
    fi
done

if [[ "$failed" -eq 0 ]]; then
    printf '\n=== Generating trace report ===\n'
    "$script_dir/trace_mod.sh"
else
    printf '\nTrace report was not generated because one or more test suites failed.\n' >&2
fi

exit "$failed"
