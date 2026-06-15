#!/bin/bash

set -euo pipefail

cd "$(dirname "$0")"

git_version=$(git log -1 --format="%h")
commit_ts=$(git log -1 --format="%ct")
commit_time=$(date -d @"$commit_ts" +"%Y-%m-%d %H:%M:%S")
current_time=$(date +"%Y-%m-%d %H:%M:%S")

version_number=$(sed -n 's/.*"version"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' version.json | tr -d '\r\n')

if [ -z "$version_number" ]; then
    echo "ERROR: failed to parse version from version.json"
    exit 1
fi

full_version="XEMA version: $git_version commit: $commit_time build: $current_time"
short_version="$version_number $git_version commit: $commit_time"

cat > ../firmware/version.h <<EOF
#ifndef _VERSION_
#define _VERSION_ "$full_version"
#define _VERSION_NUMBER_ "$short_version"
#define _VERSION_LENGTH_ 500
#define _VERSION_NUMBER_LENGTH_ 64
#endif
EOF

echo "Generated version.h:"
cat ../firmware/version.h
