#!/bin/bash

# Simple build script
# Usage: ./build.sh -l /path/to/libs [-m git BRANCH] [-j JOBS]

set -euo pipefail

LIBS_PATH=""
BUILD_MODE="local"
LIBADITOF_BRANCH="main"
JOBS=6

while [[ $# -gt 0 ]]; do
    case $1 in
        -l|--libs)
            LIBS_PATH="$2"
            shift 2
            ;;
        -m|--mode)
            BUILD_MODE="$2"
            shift 2
            ;;
        -j|--jobs)
            JOBS="$2"
            shift 2
            ;;
        *)
            if [ "$BUILD_MODE" = "git" ]; then
                LIBADITOF_BRANCH="$1"
            fi
            shift
            ;;
    esac
done

if [ -z "$LIBS_PATH" ]; then
    echo "Error: -l /path/to/libs required"
    exit 1
fi

echo "Building Docker image..."
echo "Mode: $BUILD_MODE"
if [ "$BUILD_MODE" = "git" ]; then
    echo "Branch: $LIBADITOF_BRANCH"
fi

if [ "$BUILD_MODE" = "local" ]; then
    bash build.sh -l "$LIBS_PATH" -j "$JOBS"
else
    bash build.sh -l "$LIBS_PATH" -m git "$LIBADITOF_BRANCH" -j "$JOBS"
fi

if [ $? -eq 0 ]; then
    echo "✓ Build successful!"
else
    echo "✗ Build failed!"
    exit 1
fi
