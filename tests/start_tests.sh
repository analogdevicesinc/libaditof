#!/usr/bin/env bash
set -euo pipefail

# Enter the docker folder, run cleanup + build, then tests on success.
# Always return to the previous folder.

# Default values
repeat_count=1
csv_file=""
final_test_results_path=""

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
        -o|--output)
            final_test_results_path="$2"
            shift 2
            ;;
        -h|--help)
            printf '%s\n' "Usage: $0 -f|--file CSV_FILE -o|--output PATH [-r|--repeat COUNT]"
            printf '%s\n' "  -f, --file CSV_FILE    CSV test list file (required)"
            printf '%s\n' "  -o, --output PATH      Output folder for final results script (required)"
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

# Check if output path was provided
if [[ -z "$final_test_results_path" ]]; then
    printf '%s\n' "Error: Output path is required. Use -o or --output to specify the results folder."
    printf '%s\n' "Use -h or --help for usage information"
    exit 1
fi

# Convert CSV file to absolute path
csv_file="$(realpath "$csv_file")"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ts=$(date -u +%Y%m%d_%H%M%S)
output_foldername=results_$(basename "$csv_file" .csv)_${ts}
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
    archive_path="/tmp/${output_foldername}.tar.gz"
    tar -czf "$archive_path" -C "$(dirname "$output_path")" "$(basename "$output_path")" 2>/dev/null
    
    printf '%s\n' "Archive created: ${archive_path}"
    
    # Generate self-extracting script
    generated_script="${final_test_results_path}/${output_foldername}.sh"
    mkdir -p "$final_test_results_path"
    
    cat > "$generated_script" << 'SCRIPT_EOF'
#!/bin/bash
# Self-extracting test results script

if [[ "$1" == "--extract" ]] || [[ "$1" == "-x" ]]; then
    # Extract the embedded tgz
    output_file="${2:-__DEFAULT_OUTPUT_FILE__}"
    sed -n '/^__ARCHIVE_BELOW__$/,$p' "$0" | tail -n +2 | base64 -d > "$output_file"
    echo "Extracted to $output_file"
    
    # Unarchive the tgz file
    extract_dir="${output_file%.tar.gz}"
    mkdir -p "$extract_dir"
    tar -xzf "$output_file" -C "$extract_dir"
    echo "Unarchived to $extract_dir"
    
    # Remove the tar.gz file after extraction
    rm "$output_file"
    exit 0
fi

# Default: print the test log
cat << 'EOF'
SCRIPT_EOF
    
    # Append the log content
    cat "$script_log_libaditof" >> "$generated_script"
    
    # Close the EOF marker and add extraction section
    cat >> "$generated_script" << 'SCRIPT_EOF'
EOF
exit 0

__ARCHIVE_BELOW__
SCRIPT_EOF
    
    # Append base64 encoded archive
    base64 "$archive_path" >> "$generated_script"
    
    # Make it executable
    chmod +x "$generated_script"
    
    # Replace placeholder with actual output filename
    sed -i "s/__DEFAULT_OUTPUT_FILE__/${output_foldername}.tar.gz/" "$generated_script"
    
    printf '%s\n' "Self-extracting script: ${generated_script}"
    printf '%s\n' "=========================================="
}

# Use pushd/popd to return automatically, and print results path on exit
pushd "$script_dir/docker" > /dev/null
trap 'exec 1>&3 2>&4;print_results_path;./cleanup.sh > /dev/null; popd > /dev/null;' EXIT

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
printf '%b\n' "Test executing: ./run_tests.sh -f ${csv_file} -o ${output_path} -n ${repeat_count}\n"
./run_tests.sh -f "$csv_file" -o ${output_path} -n ${repeat_count}
