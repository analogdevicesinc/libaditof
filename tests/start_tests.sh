#!/usr/bin/env bash
set -euo pipefail

# Enter the docker folder, run cleanup + build, then tests on success.
# Always return to the previous folder.

# Default values
repeat_count=1
csv_file=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -r|--repeat)
            repeat_count="$2"
            shift 2
            ;;
        -f|--file)
            csv_file="$2"
            shift 2
            ;;
        -h|--help)
            printf '%s\n' "Usage: $0 -f|--file CSV_FILE [-r|--repeat COUNT]"
            printf '%s\n' "  -f, --file CSV_FILE    CSV test list file (required)"
            printf '%s\n' "  -r, --repeat COUNT     Number of times to run tests (default: 1)"
            printf '%s\n' "  -h, --help             Show this help message"
            exit 0
            ;;
        *)
            printf '%s\n' "Unknown option: $1"
            printf '%s\n' "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Check if CSV file was provided
if [[ -z "$csv_file" ]]; then
    printf '%s\n' "Error: CSV file is required. Use -f or --file to specify a test list."
    printf '%s\n' "Use -h or --help for usage information"
    exit 1
fi

# Convert CSV file to absolute path
csv_file="$(realpath "$csv_file")"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ts=$(date -u +%Y%m%d_%H%M%S)
output_foldername=results_${ts}
output_path="/tmp/${output_foldername}"
# Create output directory and setup logging
if [[ -n "$output_path" ]]; then
    mkdir -p "$output_path"
    script_log_docker="${output_path}/docker_summary.log"
	script_log_libaditof="${output_path}/libaditof_summary.log"
    # Redirect all output (stdout and stderr) to both file and screen
fi

# Function to print results path on exit
print_results_path() {
    printf '\n'
    printf '%s\n' "=========================================="
    printf '%s\n' "Test results location: ${output_path}"
    
    # Create tgz archive of results
    archive_path="./${output_foldername}.tar.gz"
    tar -czf "$archive_path" -C "$(dirname "$output_path")" "$(basename "$output_path")" 2>/dev/null
    
    printf '%s\n' "Archive created: ${archive_path}"
    printf '%s\n' "=========================================="
}

# Use pushd/popd to return automatically, and print results path on exit
pushd "$script_dir/docker" > /dev/null
trap './cleanup.sh; popd > /dev/null; print_results_path' EXIT

# Save original stdout and stderr
exec 3>&1 4>&2

# Redirect to docker log for cleanup and build
exec > >(tee -a "$script_log_docker") 2>&1

# Cleanup previous artifacts
./cleanup.sh

# Build the container (compilation happens inside build.sh)
set +e
./build.sh -l ~/dev/libs/
BUILD_RC=$?
set -e

# If build returned error code 1, exit the script
if [[ "$BUILD_RC" -eq 1 ]]; then
	printf '%s\n' "Build failed with exit code 1; aborting."
	exit 1
fi

# If build failed with any non-zero code, abort and return that code
if [[ "$BUILD_RC" -ne 0 ]]; then
	printf '%s\n' "Build failed with exit code ${BUILD_RC}; aborting."
	exit "$BUILD_RC"
fi

# Restore original stdout/stderr, then redirect to libaditof log for tests
exec 1>&3 2>&4
exec > >(tee -a "$script_log_libaditof") 2>&1

# If build succeeded, run tests
./run_tests.sh -f "$csv_file" -o ${output_path} -n ${repeat_count}
