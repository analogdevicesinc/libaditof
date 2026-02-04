#!/bin/bash

# Build and run CSV tests from docker folder
# Usage: ./build_and_test.sh -l /path/to/libs -c test_csvs/system_test_list.csv -o ./test_results [-m git BRANCH]

set -e

# Function to display help
show_help() {
    cat << EOF
Usage: $0 -l LIBS_PATH -c CSV_FILE -o OUTPUT_DIR [-m git BRANCH] [-j JOBS]

Build Docker image and run CSV tests from docker folder.

Required Options:
    -l, --libs PATH     Path to libs folder to copy into container
    -c, --csv CSV_FILE  CSV test file (relative to ../test_csvs/)
    -o, --output DIR    Output folder for test results

Optional Options:
    -m, --mode MODE     Build mode: 'local' (default) or 'git'
    BRANCH              Git branch for libaditof (only with --mode git)
    -j, --jobs N        Number of parallel build jobs (default: 6)
    -h, --help          Show this help message

Examples:
    $0 -l ../../../libs/ -c test_csvs/system_test_list.csv -o ./test_results
    $0 -l ../../../libs/ -c test_csvs/system_test_list.csv -o ./test_results -m git main
    $0 -l ../../../libs/ -c test_csvs/system_test_list.csv -o ./test_results -m git feature-x -j 4

EOF
    exit 0
}

# Default values
LIBS_PATH=""
CSV_FILE=""
OUTPUT_DIR=""
BUILD_MODE="local"
LIBADITOF_BRANCH="main"
JOBS=6

# Parse options
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            ;;
        -l|--libs)
            LIBS_PATH="$2"
            shift 2
            ;;
        -c|--csv)
            CSV_FILE="$2"
            shift 2
            ;;
        -o|--output)
            OUTPUT_DIR="$2"
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
            # Positional argument: branch for git mode
            if [ "$BUILD_MODE" = "git" ]; then
                LIBADITOF_BRANCH="$1"
            fi
            shift
            ;;
    esac
done

# Validate required parameters
if [ -z "$LIBS_PATH" ] || [ -z "$CSV_FILE" ] || [ -z "$OUTPUT_DIR" ]; then
    echo "Error: Missing required parameters."
    echo "Use -h or --help for usage information"
    exit 1
fi

echo ""
echo "=========================================="
echo "ADCAM Build and Test Script"
echo "=========================================="
echo "Build Mode: $BUILD_MODE"
if [ "$BUILD_MODE" = "git" ]; then
    echo "Libaditof Branch: $LIBADITOF_BRANCH"
fi
echo "Libs Path: $LIBS_PATH"
echo "CSV File: $CSV_FILE"
echo "Output Dir: $OUTPUT_DIR"
echo "Build Jobs: $JOBS"
echo "=========================================="
echo ""

# Step 1: Build Docker image
echo "Step 1: Building Docker image..."
if [ "$BUILD_MODE" = "local" ]; then
    bash ./build.sh -l "$LIBS_PATH" -j "$JOBS"
else
    bash ./build.sh -l "$LIBS_PATH" -m git "$LIBADITOF_BRANCH" -j "$JOBS"
fi

if [ $? -ne 0 ]; then
    echo "✗ Build failed!"
    exit 1
fi

echo ""
echo "✓ Build completed successfully!"
echo ""

# Step 2: Run tests
echo "Step 2: Running CSV tests..."
cd ..

# Verify CSV file exists
if [ ! -f "$CSV_FILE" ]; then
    echo "Error: CSV file not found: $CSV_FILE"
    exit 1
fi

# Run tests
bash ./start_tests.sh -f "$CSV_FILE" -o "$OUTPUT_DIR"

TEST_RESULT=$?

# Return to docker folder
cd docker

if [ $TEST_RESULT -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✓ Build and tests completed successfully!"
    echo "=========================================="
    echo "Results saved to: $OUTPUT_DIR"
    echo ""
else
    echo ""
    echo "=========================================="
    echo "✗ Tests failed with exit code: $TEST_RESULT"
    echo "=========================================="
    exit $TEST_RESULT
fi
