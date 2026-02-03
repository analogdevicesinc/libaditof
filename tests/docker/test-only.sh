#!/bin/bash

# Simple test script
# Usage: ./test-only.sh -c test_csvs/system_test_list.csv -o ./test_results

set -e

CSV_FILE=""
OUTPUT_DIR=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--csv)
            CSV_FILE="$2"
            shift 2
            ;;
        -o|--output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        *)
            shift
            ;;
    esac
done

if [ -z "$CSV_FILE" ] || [ -z "$OUTPUT_DIR" ]; then
    echo "Error: -c CSV_FILE and -o OUTPUT_DIR required"
    exit 1
fi

echo "Running CSV tests..."
echo "CSV: $CSV_FILE"
echo "Output: $OUTPUT_DIR"

cd ..
bash ./start_tests.sh -f "$CSV_FILE" -o "$OUTPUT_DIR"

if [ $? -eq 0 ]; then
    echo "✓ Tests completed!"
    cd docker
else
    echo "✗ Tests failed!"
    cd docker
    exit 1
fi
