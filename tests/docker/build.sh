#!/bin/bash

# Exit immediately if any command fails
set -e

# Trap errors and clean up
trap 'handle_error' ERR

handle_error() {
    local line_no=$1
    echo ""
    echo "✗ Script failed at line $line_no"
    exit 1
}

# Function to display help
show_help() {
    cat << EOF
Usage: $0 [OPTIONS]

Build a Docker container to test ADCAM repository build in a clean environment.

Options:
    -h, --help          Show this help message
    -j, --jobs N        Number of parallel build jobs (default: 6)
    -t, --tag NAME      Docker image tag name (default: adcam-build-test)
    -l, --libs PATH     Path to libs folder to copy into container (required)
    -v, --verbose       Show verbose build output
    --no-cache          Build without using Docker cache

Examples:
    $0 -l /path/to/libs                     # Build with main branches
    $0 -l /path/to/libs main main           # Explicitly specify main branches
    $0 -l /path/to/libs feature-branch main # Test a feature branch
    $0 -j 4 -l /path/to/libs main main      # Build with 4 parallel jobs
    $0 --no-cache -l /path/to/libs main main # Force clean build

The build output is saved to build_output.log
EOF
    exit 0
}

# Default values
JOBS=6
IMAGE_TAG="adcam-build-test"
DOCKER_OPTS="--progress=plain"
LIBS_PATH=""

# Parse options
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            ;;
        -j|--jobs)
            JOBS="$2"
            shift 2
            ;;
        -t|--tag)
            IMAGE_TAG="$2"
            shift 2
            ;;
        -l|--libs)
            LIBS_PATH="$2"
            shift 2
            ;;
        -v|--verbose)
            DOCKER_OPTS="$DOCKER_OPTS --no-cache"
            shift
            ;;
        --no-cache)
            DOCKER_OPTS="$DOCKER_OPTS --no-cache"
            shift
            ;;
        -*)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
        *)
            break
            ;;
    esac
done

# Check if libs path is provided
if [ -z "$LIBS_PATH" ]; then
    echo "Error: libs path is required. Use -l or --libs to specify the path."
    echo "Use -h or --help for usage information"
    exit 1
fi

# Verify libs path exists
if [ ! -d "$LIBS_PATH" ]; then
    echo "Error: libs path does not exist: $LIBS_PATH"
    exit 1
fi

# Get absolute path
LIBS_PATH=$(realpath "$LIBS_PATH")

# Copy libs to local directory for Docker context
echo "Copying libs from $LIBS_PATH to ./libs..."
rm -rf ./libs
cp -r "$LIBS_PATH" ./libs

# Handle local mode
echo "Using local code from workspace..."

# Get the workspace root (two levels up from this script)
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# Copy local code, excluding build directories and scripts
echo "Copying local workspace to ./local_code..."
rm -rf ./local_code
mkdir -p ./local_code

# Use rsync for efficient copying with exclusions
rsync -a -vv --exclude='build' \
            --exclude='libaditof/build' \
            --exclude='tests/docker' \
            --exclude='test-results' \
            --exclude='test-logs' \
            --exclude='.git' \
            --exclude='.github' \
            "$WORKSPACE_ROOT/" ./local_code/

echo "  Source: $WORKSPACE_ROOT"
echo "  Excluded: build/, libaditof/build/, test-results/, test-logs/, .git/"

DOCKERFILE="Dockerfile.local"

# Build the Docker image with output visible
echo "Building Docker image for libaditof..."
echo "  Mode: Local workspace"
echo "  Build jobs: $JOBS"
echo "  Image tag: $IMAGE_TAG"
echo "  Libs source: $LIBS_PATH"
echo "  Dockerfile: $DOCKERFILE"
echo ""

docker build \
    $DOCKER_OPTS \
    --build-arg BUILD_JOBS=$JOBS \
    -f $DOCKERFILE \
    -t $IMAGE_TAG . 2>&1 | tee build_output.log

# Capture the exit code from docker build (not from tee)
BUILD_EXIT_CODE=${PIPESTATUS[0]}

# Check for compilation errors in the build output
if [ $BUILD_EXIT_CODE -ne 0 ]; then
    echo ""
    echo "✗ Docker build failed with exit code: $BUILD_EXIT_CODE"
    echo ""
    echo "Compilation/Build Errors Found:"
    echo "================================"
    grep -i "error\|failed\|undefined reference" build_output.log | head -20 || true
    echo ""
    echo "Check build_output.log for full details"
    exit $BUILD_EXIT_CODE
fi

# Additional check: scan for compiler errors even if exit code is 0
if grep -i "error:" build_output.log > /dev/null 2>&1; then
    echo ""
    echo "⚠ Warning: Potential compilation errors detected in build output"
    echo "================================"
    grep -i "error:" build_output.log | head -10 || true
    exit 1
fi

# Build succeeded
echo ""
echo "✓ Docker image built successfully!"
echo "Full build log saved to: build_output.log"
echo ""
echo "To run the container with docker-compose (recommended):"
echo ""
echo "  1. Install docker-compose (if not already installed):"
echo "     sudo curl -L \"https://github.com/docker/compose/releases/latest/download/docker-compose-\$(uname -s)-\$(uname -m)\" -o /usr/local/bin/docker-compose"
echo "     sudo chmod +x /usr/local/bin/docker-compose"
echo ""
echo "  2. Run a command in the container with device access:"
echo "     docker-compose run -v "$(pwd)/out:/out" -w /out aditof /workspace/libaditof/build/tests/sdk/bin/camera-adsd3500_reset"
echo "     docker-compose run -v "$(pwd)/out:/out" -w /out aditof /workspace/libaditof/build/tests/sdk/bin/camera-get_frames --mode=3"
echo ""
echo "  3. Or, get an interactive shell:"
echo "     docker-compose run aditof /bin/bash"
echo ""
echo "Alternative: Run directly with docker:"
echo "  docker run -it --device /dev/media0 --device /dev/video0 $IMAGE_TAG /bin/bash"
