#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

python3 "$repo_root/mod/_test/framework/tools/trace_report.py" \
    --search-root "$repo_root" \
    --req-format '^\s*(RQ\.MOD\.[0-9]{5})\s+-\s+(.+?)\s*$' \
    --spec-files '(^|/)mod/_reqs/[^/]+\.md$' \
    --results-files '(^|/)mod/_test/mod/(analysis_results\.md|[^/]+/results\.txt)$'
