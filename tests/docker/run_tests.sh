#!/usr/bin/env bash
#
# process_tsv_printf.sh
# Usage: process_tsv_printf.sh [-f FILE] [FILE]
#        process_tsv_printf.sh -h | --help
#
# Reads a TSV with header: TestID<TAB>Execute<TAB>TestName<TAB>TestParameters
# Processes each row and runs docker-compose per-test when Execute is Y/y.
#

#set -x
# If you prefer strict mode, enable it but handle command failures explicitly:
# set -euo pipefail

# Setup colors (tput when available; fallback to ANSI)
if tput setaf 1 &>/dev/null; then
  RED=$(tput setaf 1)
  GREEN=$(tput setaf 2)
  WHITE=$(tput setaf 7)
  # 208 is an orange-ish index on many 256-color palettes; tput may fail if 256 colors unsupported.
  ORANGE=$(tput setaf 208 2>/dev/null || true)
  RESET=$(tput sgr0)
else
  # fallback to ANSI sequences
  RED=$'\033[31m'; GREEN=$'\033[32m'; WHITE=$'\033[37m'; ORANGE=''; RESET=$'\033[0m'
fi

print_usage() {
    printf '%b\n' "${WHITE}Usage:${RESET} $0 [-f FILE|--file FILE] [FILE]"
    printf '%b\n' "  -f, --file FILE    Path to CSV file to process (default: test_list.csv)"
    printf '%b\n' "  -o, --output PATH  Local path for results"
    printf '%b\n' "  -h, --help         Show this help message and exit"
    printf '\n'
    printf '%b\n' "Expect a TSV with header line, e.g.:"
    printf '%b\n' "  TestID<TAB>Execute<TAB>TestName<TAB>TestParameters"
    printf '%b\n' "The script ignores the header and starts processing from the second line."
}

# Default input file
input_file=""
output_path=""

# If the user provided no arguments at all, show help and exit.
# This avoids silently using the default file and makes the script usage explicit.
if [[ $# -eq 0 ]]; then
    print_usage
    exit 0
fi

# Parse arguments (support -h/--help and -f/--file and positional)
while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            print_usage
            exit 0
            ;;
        -f|--file)
            if [[ -n "${2-}" && "${2:0:1}" != "-" ]]; then
                input_file="$2"
                shift 2
            else
                printf '%b\n' "${RED}Error:${RESET} --file requires a non-empty argument."
                print_usage
                exit 2
            fi
            ;;
        -o|--output)
            if [[ -n "${2-}" && "${2:0:1}" != "-" ]]; then
                output_path="$2"
                shift 2
            else
                printf '%b\n' "${RED}Error:${RESET} --output requires a non-empty argument."
                print_usage
                exit 2
            fi
            ;;
        --file=*)
            input_file="${1#*=}"
            shift
            ;;
        --) # end of options
            shift
            break
            ;;
        -*)
            printf '%b\n' "${RED}Unknown option:${RESET} $1"
            print_usage
            exit 2
            ;;
        *)
            # positional argument — treat as input file if provided
            if [[ -z "${positional_given:-}" ]]; then
                input_file="$1"
                positional_given=1
                shift
            else
                # extra positional args ignored for now
                shift
            fi
            ;;
    esac
done

if [[ ! -f "$input_file" ]]; then
    printf '%b\n' "${RED}Error:${RESET} '$input_file' not found."
    printf '\n'
    print_usage
    exit 1
fi

TESTS_TOTAL=0
TESTS_SKIPPED=0
TESTS_PASSED=0
TESTS_FAILED=0

printf '%s\n' "--- Processing ---"

while IFS=$',' read -r testid execute testname testparameters || \
      [[ -n "$testid" || -n "$execute" || -n "$testname" || -n "$testparameters" ]]; do

    # skip empty lines
    if [[ -z "$testid" && -z "$execute" && -z "$testname" && -z "$testparameters" ]]; then
        continue
    fi

    # Print the initial "Processing" text without a newline, using printf (portable)
    printf '%s' "${testid},${testname},${testparameters},"
    ((TESTS_TOTAL++))
    if [[ "$execute" != [Yy] ]]; then
        printf '%b\n' "${ORANGE}Skipped${RESET}"
        ((TESTS_SKIPPED++))
    else
        mkdir -p "${output_path}/test_${testid}"

        REMOTE_FOLDER="/out/test_${testid}"
        LOCAL_DIR="${output_path}/test_${testid}"
        FOLDER_MAPPING="${LOCAL_DIR}:${REMOTE_FOLDER}"
        LOGPATH="${LOCAL_DIR}/result.log"

        # Prevent docker-compose from reading the loop's stdin by redirecting it from /dev/null.
        # Capture exit code so the script doesn't unexpectedly exit if you have set -e.
        docker-compose run --rm -T \
            -v "$FOLDER_MAPPING" \
            -w "$REMOTE_FOLDER" \
            aditof /workspace/libaditof/build/tests/sdk/bin/"$testname" $testparameters \
            > "$LOGPATH" 2>&1 < /dev/null
        rc=$?
        if [[ $rc -ne 0 ]]; then
            # Print a clear message using printf (use %b if you want colors interpreted)
            printf '%b\n' "${RED}docker-compose exited with code ${rc} for TestID ${testid}${RESET}"
        else
            # docker-compose succeeded — print success on the same line (we printed the "Processing..." without newline)
            pattern='PASSED'

            # Check the generated log for the pattern
            if grep -q -- "$pattern" "$LOGPATH"; then
                found=true
            else
                found=false
            fi

            # use it:
            if $found; then
                printf '%b\n' "${GREEN}Passed${RESET}"
                ((TESTS_PASSED++))
            else
                printf '%b\n' "${RED}Failed${RESET}"
                ((TESTS_FAILED++))
            fi
        fi
    fi

done < <(sed 's/\r$//' "$input_file" | tail -n +2)

printf '%s\n' "--- Summary of Test Results ---"
printf '%s\n' "Processed,${TESTS_TOTAL}"
printf '%s\n' "Skipped,${TESTS_SKIPPED}"
printf '%s\n' "Passed,${TESTS_PASSED}"
printf '%s\n' "Failed,${TESTS_FAILED}"