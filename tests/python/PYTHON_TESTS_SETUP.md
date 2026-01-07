# Python Bindings Test Setup

Test helper scripts and documentation for the Python bindings are located in `tests/python/`.

## Scripts

Both helper scripts are located in `tests/python/`:

### `tests/python/setup_python_test_env.sh`
Comprehensive setup script that:
- Checks Python installation
- Installs Python test dependencies (pytest, numpy, matplotlib)
- Builds the project with Python bindings enabled
- Sets up the test environment
- Runs the test suite

**Usage:**
```bash
./tests/python/setup_python_test_env.sh [options]
```

**Options:**
- `-h, --help` - Show help message
- `-c, --clean` - Start with a clean build directory
- `-b, --build-only` - Only build, don't run tests
- `-t, --test-only` - Only run tests (requires previous build)
- `-v, --verbose` - Verbose output
- `--nvidia` - Build for NVIDIA target
- `--nxp` - Build for NXP target

**Examples:**
```bash
# Full setup and test
./setup_python_test_env.sh

# Clean build and test
./setup_python_test_env.sh -c

# Build only
./setup_python_test_env.sh -b

# Test only (after previous build)
./setup_python_test_env.sh -t

# Verbose output for debugging
./setup_python_test_env.sh -v

# Build for NVIDIA target and test
./setup_python_test_env.sh --nvidia
```

### `tests/python/run_python_tests.sh`
Quick test runner for projects already built. Useful for iterative testing during development.

**Usage:**
```bash
./tests/python/run_python_tests.sh [pytest_args]
```

**Examples:**
```bash
# Run all tests
./run_python_tests.sh

# Verbose output
./run_python_tests.sh -v

# Run specific test class
./run_python_tests.sh tests/python/test_aditof_bindings.py::TestVersionAPI

# Run tests matching pattern
./run_python_tests.sh -k "camera"

# Stop on first failure
./run_python_tests.sh -x

# Show print statements
./run_python_tests.sh -s
```

## Quick Start

### First Time Setup
```bash
cd /path/to/libaditof
./tests/python/setup_python_test_env.sh
```

This will:
1. Install Python dependencies
2. Build the project
3. Run all tests
4. Display a summary

### After Making Changes
```bash
# Option 1: Quick test (if only test code changed)
./tests/python/run_python_tests.sh

# Option 2: Rebuild and test
./tests/python/setup_python_test_env.sh -t
```

## Environment Variables

The scripts automatically set:
- `PYTHONPATH` - Points to the build directory where aditofpython is located

You can also manually set these:
```bash
export PYTHONPATH="$(pwd)/build:$(pwd)/build/sdk:$PYTHONPATH"
```

## Requirements

- Python 3.6+
- CMake 3.12+
- C++ compiler (gcc, clang, or MSVC)

### Python Packages
Installed automatically by the setup script:
- pytest >= 7.0
- numpy >= 1.20
- matplotlib >= 3.3

Install manually with:
```bash
pip install pytest numpy matplotlib
```

## Troubleshooting

### "aditofpython not found"
Make sure you've run the setup script to build the Python bindings:
```bash
./tests/python/setup_python_test_env.sh
```

### "pytest not found"
Install pytest:
```bash
pip install pytest
```

### Build fails
Try a clean build:
```bash
./tests/python/setup_python_test_env.sh -c
```

### Tests are skipped
This is normal if camera hardware is unavailable. Tests gracefully skip when hardware dependencies are missing. Run with verbose output to see skip reasons:
```bash
./tests/python/run_python_tests.sh -v -rs
```

## CI/CD Integration

These scripts can be used in CI/CD pipelines:

```bash
#!/bin/bash
set -e
cd /path/to/libaditof
./tests/python/setup_python_test_env.sh
```

Or for faster CI (if build is in separate step):
```bash
#!/bin/bash
set -e
cd /path/to/libaditof
./tests/python/setup_python_test_env.sh -t
```

## Development Workflow

Typical development workflow:

```bash
# Initial setup
./tests/python/setup_python_test_env.sh

# Edit tests...

# Quick test loop
./tests/python/run_python_tests.sh -v

# Test specific test class
./tests/python/run_python_tests.sh tests/python/test_aditof_bindings.py::TestSystemAPI -v

# When satisfied, rebuild everything
./tests/python/setup_python_test_env.sh -c
```

## More Information

- See [tests/python/README.md](tests/python/README.md) for detailed test documentation
- See [PYTHON_TESTING_SETUP.md](PYTHON_TESTING_SETUP.md) for architecture overview
