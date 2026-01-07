#!/bin/bash

################################################################################
# Quick Python Test Runner
#
# Fast script to run Python binding tests with sensible defaults
# This script assumes the project is already built
#
# Usage:
#   ./run_python_tests.sh [pytest_args]
#
# Examples:
#   ./run_python_tests.sh                    # Run all tests
#   ./run_python_tests.sh -v                 # Verbose output
#   ./run_python_tests.sh -k "version"       # Run tests matching pattern
#   ./run_python_tests.sh -x                 # Stop on first failure
################################################################################

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BUILD_DIR="${SCRIPT_DIR}/build"

# Check if build directory exists
if [ ! -d "$BUILD_DIR" ]; then
    echo "Error: Build directory not found at $BUILD_DIR"
    echo "Please run: ./setup_python_test_env.sh"
    exit 1
fi

# Check if aditofpython module exists
if [ ! -f "$BUILD_DIR/sdk/aditofpython.so" ] && [ ! -f "$BUILD_DIR/sdk/aditofpython.pyd" ]; then
    echo "Warning: aditofpython module not found"
    echo "The project may not have been built with Python bindings enabled"
    echo "Run: ./setup_python_test_env.sh"
fi

# Set up Python path
export PYTHONPATH="${BUILD_DIR}:${BUILD_DIR}/sdk:$PYTHONPATH"

echo "Python Test Environment"
echo "======================="
echo "Python: $(python3 --version)"
echo "pytest: $(pytest --version 2>/dev/null || echo 'not installed')"
echo "PYTHONPATH: $PYTHONPATH"
echo ""

# Run pytest with any provided arguments
echo "Running tests..."
pytest tests/python/ "$@"
