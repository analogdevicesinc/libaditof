#!/usr/bin/env bash
set -euo pipefail

# Script to clone, build, and run full test suite for libaditof
# Usage: ./run_test_suite.sh branch_or_tag output_dir csv1 [csv2] [csv3] ...

# Check minimum arguments
if [ $# -lt 3 ]; then
    printf '%s\n' "Error: Insufficient arguments."
    printf '%s\n' "Usage: $0 branch_or_tag output_dir csv1 [csv2] [csv3] ..."
    printf '%s\n' "Example: $0 gtest-integration ~/Documents test_csvs/test_set_1.csv test_csvs/test_set_2.csv"
    exit 1
fi

# Parse arguments
BRANCH="$1"
OUTPUT_DIR="$2"
REPO_URL="https://github.com/analogdevicesinc/libaditof"

# Create working directory name with branch and random number
BRANCH_SAFE="${BRANCH//\//_}"  # Replace / with _
RANDOM_NUM="$RANDOM"
WORK_DIR="${WORK_DIR:-/tmp/libaditof_${BRANCH_SAFE}_${RANDOM_NUM}}"

# Shift to get CSV files from remaining arguments
shift 2
CSV_FILES=("$@")

# Save original directory to return to later
ORIGINAL_DIR="$(pwd)"

# Set trap to always return to original directory on exit
trap 'cd "$ORIGINAL_DIR"' EXIT

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

printf '%b\n' "${GREEN}=== ADCAM Test Suite Runner ===${NC}"
printf '%s\n' "Branch/Tag: $BRANCH"
printf '%s\n' "Output Directory: $OUTPUT_DIR"
printf '%s\n' "Working Directory: $WORK_DIR"
printf '%s\n' "CSV Files: ${CSV_FILES[*]}"
printf '\n'

# Create and enter working directory
mkdir -p "$WORK_DIR"
cd "$WORK_DIR"

# Step 1-2: Clone repository (if not already present)
if [ ! -d "libaditof" ]; then
    printf '%b\n' "${YELLOW}Cloning repository...${NC}"
    git clone "$REPO_URL"
else
    printf '%b\n' "${YELLOW}Repository already exists, updating...${NC}"
    cd libaditof
    git fetch --all
    cd ..
fi

cd libaditof

# Step 3: Checkout specified branch or tag
printf '%b\n' "${YELLOW}Checking out $BRANCH...${NC}"
git checkout "$BRANCH"

# Step 4: Initialize gtest submodule
printf '%b\n' "${YELLOW}Initializing gtest submodule...${NC}"
git submodule update --init dependencies/third-party/gtest

# Step 5-6: Enter tests directory
cd tests

# Convert CSV file paths to absolute paths
for i in "${!CSV_FILES[@]}"; do
    CSV_FILES[$i]="$(realpath "${CSV_FILES[$i]}")"
done

# Iterate over CSV files, building container on first run
for i in "${!CSV_FILES[@]}"; do
    csv_file="${CSV_FILES[$i]}"
    test_num=$((i + 1))
    
    if [ $i -eq 0 ]; then
        # First CSV: build container
        printf '%b\n' "${GREEN}Running test set $test_num (building container): $csv_file${NC}"
        ./start_tests.sh -f "$csv_file" -o "$OUTPUT_DIR" -b
    elif [ $i -eq $((${#CSV_FILES[@]} - 1)) ]; then
        # Last CSV: reuse container and cleanup after
        printf '%b\n' "${GREEN}Running test set $test_num (reusing container, cleanup after): $csv_file${NC}"
        ./start_tests.sh -f "$csv_file" -o "$OUTPUT_DIR" -c
    else
        # Middle CSVs: reuse container
        printf '%b\n' "${GREEN}Running test set $test_num (reusing container): $csv_file${NC}"
        ./start_tests.sh -f "$csv_file" -o "$OUTPUT_DIR"
    fi
done

printf '\n'
printf '%b\n' "${GREEN}=== Test Suite Complete ===${NC}"
printf '%s\n' "Results are available in: $OUTPUT_DIR"
printf '%s\n' "Results files:"
ls -1t "$OUTPUT_DIR"/results_*.sh 2>/dev/null | head -3 || printf '%s\n' "No result files found"