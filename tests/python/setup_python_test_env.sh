#!/bin/bash

################################################################################
# Setup script for testing Python bindings
#
# This script creates a clean environment for testing the aditofpython bindings
# including installing dependencies, building the project, and running tests.
#
# Usage:
#   ./setup_python_test_env.sh [options]
#
# Options:
#   -h, --help              Show this help message
#   -c, --clean             Start with a clean build directory
#   -b, --build-only        Only build, don't run tests
#   -t, --test-only         Only run tests (requires previous build)
#   -v, --verbose           Verbose output
#   --nvidia                Build for NVIDIA target
#   --nxp                   Build for NXP target
################################################################################

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default options
CLEAN_BUILD=false
BUILD_ONLY=false
TEST_ONLY=false
VERBOSE=false
NVIDIA_TARGET=false
NXP_TARGET=false
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Functions
print_usage() {
    grep "^#" "$0" | grep -E "^\s*#" | head -30
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            print_usage
            exit 0
            ;;
        -c|--clean)
            CLEAN_BUILD=true
            shift
            ;;
        -b|--build-only)
            BUILD_ONLY=true
            shift
            ;;
        -t|--test-only)
            TEST_ONLY=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        --nvidia)
            NVIDIA_TARGET=true
            shift
            ;;
        --nxp)
            NXP_TARGET=true
            shift
            ;;
        *)
            print_error "Unknown option: $1"
            print_usage
            exit 1
            ;;
    esac
done

# Main script
main() {
    print_info "Python Bindings Test Environment Setup"
    echo ""

    # Check if we're in the right directory
    if [ ! -f "CMakeLists.txt" ]; then
        print_error "CMakeLists.txt not found. Please run this script from the project root."
        exit 1
    fi

    # Step 1: Check Python installation
    print_info "Checking Python installation..."
    if ! command -v python3 &> /dev/null; then
        print_error "Python 3 is not installed"
        exit 1
    fi
    PYTHON_VERSION=$(python3 --version)
    print_success "Found $PYTHON_VERSION"
    echo ""

    # Step 2: Install Python test dependencies
    print_info "Installing Python test dependencies..."
    if [ "$VERBOSE" = true ]; then
        pip install -q pytest numpy matplotlib
    else
        pip install -q pytest numpy matplotlib 2>/dev/null || {
            print_warning "Some packages may need to be installed manually"
            pip install pytest numpy matplotlib
        }
    fi
    print_success "Python dependencies installed"
    echo ""

    # Step 3: Clean build directory if requested
    if [ "$CLEAN_BUILD" = true ]; then
        print_info "Cleaning build directory..."
        rm -rf build
        print_success "Build directory cleaned"
        echo ""
    fi

    # Step 4: Create build directory
    if [ ! -d "build" ]; then
        print_info "Creating build directory..."
        mkdir -p build
    fi

    # Step 5: Build the project
    if [ "$TEST_ONLY" = false ]; then
        print_info "Configuring CMake..."
        cd build

        # Build cmake arguments
        CMAKE_ARGS="-DBUILD_TESTING=ON -DWITH_PYTHON=ON"

        if [ "$NVIDIA_TARGET" = true ]; then
            CMAKE_ARGS="$CMAKE_ARGS -DNVIDIA=1"
            print_info "Building for NVIDIA target"
        elif [ "$NXP_TARGET" = true ]; then
            CMAKE_ARGS="$CMAKE_ARGS -DNXP=1"
            print_info "Building for NXP target"
        fi

        if [ "$VERBOSE" = true ]; then
            cmake $CMAKE_ARGS ..
        else
            cmake $CMAKE_ARGS .. > /dev/null
        fi
        print_success "CMake configured"
        echo ""

        print_info "Building project..."
        if [ "$VERBOSE" = true ]; then
            make -j$(nproc)
        else
            make -j$(nproc) > /dev/null 2>&1
        fi
        print_success "Project built"
        echo ""

        cd ..
    else
        # Verify build directory exists for test-only mode
        if [ ! -d "build" ]; then
            print_error "Build directory not found. Run without --test-only to build first."
            exit 1
        fi
    fi

    # Step 6: Set up environment
    print_info "Setting up test environment..."
    export PYTHONPATH="${SCRIPT_DIR}/build:${SCRIPT_DIR}/build/sdk:$PYTHONPATH"
    print_success "PYTHONPATH set: $PYTHONPATH"
    echo ""

    # Step 7: Run tests
    if [ "$BUILD_ONLY" = false ]; then
        print_info "Running Python binding tests..."
        echo ""

        # Check if pytest is available
        if ! command -v pytest &> /dev/null; then
            print_error "pytest is not installed. Install it with: pip install pytest"
            exit 1
        fi

        # Run pytest with verbose output
        if [ "$VERBOSE" = true ]; then
            pytest tests/python/ -v -s
        else
            pytest tests/python/ -v
        fi

        # Capture exit code
        PYTEST_EXIT_CODE=$?

        echo ""
        if [ $PYTEST_EXIT_CODE -eq 0 ]; then
            print_success "All tests passed!"
        else
            print_warning "Some tests failed or were skipped (exit code: $PYTEST_EXIT_CODE)"
        fi

        # Step 8: Summary
        print_info "Test Environment Summary"
        echo "=========================="
        echo "Python: $PYTHON_VERSION"
        echo "pytest: $(pytest --version 2>/dev/null || echo 'not found')"
        echo "numpy: $(python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null || echo 'not found')"
        echo "matplotlib: $(python3 -c 'import matplotlib; print(matplotlib.__version__)' 2>/dev/null || echo 'not found')"
        echo "Build directory: ${SCRIPT_DIR}/build"
        echo "PYTHONPATH: $PYTHONPATH"
        echo ""

        if [ $PYTEST_EXIT_CODE -eq 0 ]; then
            print_success "Test environment is ready and all tests passed!"
        else
            print_warning "Test environment is ready, but some tests did not pass"
            print_info "To debug, try running with verbose mode: $0 -v"
        fi

        exit $PYTEST_EXIT_CODE
    else
        print_success "Build completed successfully!"
        echo ""
        print_info "To run tests, execute:"
        echo "  export PYTHONPATH=\"${SCRIPT_DIR}/build:${SCRIPT_DIR}/build/sdk:\$PYTHONPATH\""
        echo "  pytest tests/python/ -v"
    fi
}

# Run main function
main
