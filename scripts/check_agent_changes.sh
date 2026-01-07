#!/bin/bash
# Check that major diffs also update docs/Agent_Changes.md
#
# Usage:
#   scripts/check_agent_changes.sh [BASE] [THRESHOLD]
#
# BASE defaults to HEAD. THRESHOLD defaults to 200 (added+deleted lines).

set -euo pipefail

BASE="${1:-HEAD}"
THRESHOLD="${2:-200}"
LOG_FILE="docs/Agent_Changes.md"

if ! git rev-parse --git-dir >/dev/null 2>&1; then
    echo "ERROR: not in a git repository" >&2
    exit 2
fi

numstat="$(git diff --numstat "${BASE}")"
if [ -z "${numstat}" ]; then
    exit 0
fi

total_lines="$(
    echo "${numstat}" | awk '
    {
        add=$1; del=$2
        if (add == "-") add=0
        if (del == "-") del=0
        sum += add + del
    }
    END { print sum + 0 }'
)"

if [ "${total_lines}" -lt "${THRESHOLD}" ]; then
    exit 0
fi

if git diff --name-only "${BASE}" | grep -qx "${LOG_FILE}"; then
    exit 0
fi

cat >&2 <<EOF
ERROR: Large change detected (${total_lines} lines; threshold ${THRESHOLD}),
but ${LOG_FILE} was not updated.

Add a short entry describing the change and validation.
EOF
exit 1
