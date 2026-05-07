#!/bin/bash
#
# MIT License
# Copyright (c) 2026 Analog Devices, Inc.
#
# Run ADCAM tests in batches with delays to allow hardware reset between groups.
# This prevents hardware initialization failures when running many tests back-to-back.
#
# Usage:
#   ./libaditof/scripts/run_tests_batched.sh [options]
#   # Or from libaditof directory:
#   cd libaditof && ./scripts/run_tests_batched.sh [options]
#
# Options:
#   -d, --delay SECONDS     Delay between test batches (default: 2)
#   -b, --build-dir DIR     Build directory (default: ../../build)
#   -f, --filter PATTERN    Only run tests matching pattern
#   -v, --verbose           Show detailed output
#   -h, --help              Show this help message
#

set -e  # Exit on error

# Default configuration
DELAY=2
BUILD_DIR="../../build"  # Relative to libaditof directory
FILTER=""
VERBOSE=0
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIBADITOF_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--delay)
            DELAY="$2"
            shift 2
            ;;
        -b|--build-dir)
            BUILD_DIR="$2"
            shift 2
            ;;
        -f|--filter)
            FILTER="$2"
            shift 2
            ;;
        -v|--verbose)
            VERBOSE=1
            shift
            ;;
        -h|--help)
            grep '^#' "$0" | sed 's/^# \?//'
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Ensure build directory exists
if [[ ! -d "$BUILD_DIR" ]]; then
    echo -e "${RED}Error: Build directory not found: $BUILD_DIR${NC}"
    exit 1
fi

cd "$BUILD_DIR"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}ADCAM Batched Test Runner${NC}"
echo -e "${BLUE}========================================${NC}"
echo "Build directory: $BUILD_DIR"
echo "Delay between batches: ${DELAY}s"
if [[ -n "$FILTER" ]]; then
    echo "Filter pattern: $FILTER"
fi
echo ""

# Define test batches (logical groupings to minimize hardware conflicts)
declare -a TEST_BATCHES=(
    # Batch 1: All non-hardware unit tests (frame operations, SDK structures, status codes)
    "VersionTest|FrameTest|StatusTest|FrameTypeTest|FrameDetailsTest|FrameDataDetailsTest|FrameOperationsTest|MetadataTest|IntrinsicParametersTest|Adsd3500StatusTest|CameraDetailsTest"
    
    # Batch 2: All Camera API tests (64 tests covering all public methods)
    "CameraApiTest"
    
    # Batch 3: System and Security tests
    "SystemTest|SecurityTest"
    
    # Batch 4: Integration tests (parameters, frame capture, hardware reset, end-to-end)
    "parameter|CameraGetFrames|adsd3500SoftReset|IntegrationTest"
    
    # Batch 5: Python bindings tests
    "PythonBindingsTests"
)

# Statistics
TOTAL_BATCHES=${#TEST_BATCHES[@]}
PASSED_BATCHES=0
FAILED_BATCHES=0
SKIPPED_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

# Run tests batch by batch
for i in "${!TEST_BATCHES[@]}"; do
    BATCH_NUM=$((i + 1))
    PATTERN="${TEST_BATCHES[$i]}"
    
    # Apply filter if specified
    if [[ -n "$FILTER" ]] && [[ ! "$PATTERN" =~ $FILTER ]]; then
        echo -e "${YELLOW}[Batch $BATCH_NUM/$TOTAL_BATCHES] Skipping (filtered): $PATTERN${NC}"
        continue
    fi
    
    echo -e "${BLUE}[Batch $BATCH_NUM/$TOTAL_BATCHES] Running: $PATTERN${NC}"
    
    # Run the test batch
    if [[ $VERBOSE -eq 1 ]]; then
        OUTPUT=$(ctest -R "$PATTERN" --output-on-failure 2>&1)
        RESULT=$?
        echo "$OUTPUT"
    else
        OUTPUT=$(ctest -R "$PATTERN" --output-on-failure 2>&1)
        RESULT=$?
    fi
    
    # Parse results
    if echo "$OUTPUT" | grep -q "No tests were found"; then
        echo -e "${YELLOW}  └─ No tests found for pattern${NC}"
    else
        # CTest outputs: "X% tests passed, Y tests failed out of Z"
        TESTS_TOTAL=$(echo "$OUTPUT" | grep -oP '(?<=out of )\d+' | head -1 | tr -d '\n')
        TESTS_FAILED=$(echo "$OUTPUT" | grep -oP '\d+(?= tests? failed)' | head -1 | tr -d '\n')
        TESTS_SKIPPED=$(echo "$OUTPUT" | grep -c "Skipped" | tr -d '\n')
        
        # Ensure variables are valid integers
        TESTS_TOTAL=${TESTS_TOTAL:-0}
        TESTS_FAILED=${TESTS_FAILED:-0}
        TESTS_SKIPPED=${TESTS_SKIPPED:-0}
        
        # Calculate passed tests
        TESTS_RUN=$((TESTS_TOTAL - TESTS_FAILED))
        
        if [[ $RESULT -eq 0 ]]; then
            echo -e "${GREEN}  ✓ Passed: $TESTS_RUN test(s)${NC}"
            PASSED_BATCHES=$((PASSED_BATCHES + 1))
            PASSED_TESTS=$((PASSED_TESTS + TESTS_RUN))
        else
            echo -e "${RED}  ✗ Failed: $TESTS_FAILED test(s)${NC}"
            if [[ $VERBOSE -eq 0 ]]; then
                echo "$OUTPUT" | grep -A 5 "FAILED"
            fi
            FAILED_BATCHES=$((FAILED_BATCHES + 1))
            FAILED_TESTS=$((FAILED_TESTS + TESTS_FAILED))
        fi
        
        if [[ $TESTS_SKIPPED -gt 0 ]]; then
            echo -e "${YELLOW}  ⊘ Skipped: $TESTS_SKIPPED test(s)${NC}"
            SKIPPED_TESTS=$((SKIPPED_TESTS + TESTS_SKIPPED))
        fi
    fi
    
    # Delay between batches (except after last batch)
    if [[ $BATCH_NUM -lt $TOTAL_BATCHES ]]; then
        echo -e "${BLUE}  ⏱  Waiting ${DELAY}s for hardware cooldown...${NC}"
        sleep "$DELAY"
        echo ""
    fi
done

# Print summary
echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Test Summary${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "Total batches run: $TOTAL_BATCHES"
echo -e "${GREEN}Passed batches: $PASSED_BATCHES${NC}"
echo -e "${RED}Failed batches: $FAILED_BATCHES${NC}"
echo ""
echo -e "${GREEN}Passed tests: $PASSED_TESTS${NC}"
echo -e "${RED}Failed tests: $FAILED_TESTS${NC}"
echo -e "${YELLOW}Skipped tests: $SKIPPED_TESTS${NC}"
echo ""

# Exit with appropriate code
if [[ $FAILED_BATCHES -gt 0 ]]; then
    echo -e "${RED}❌ Some test batches failed${NC}"
    exit 1
else
    echo -e "${GREEN}✅ All test batches passed${NC}"
    exit 0
fi
