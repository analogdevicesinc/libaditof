"""
Pytest configuration and shared fixtures for aditofpython tests.
"""
import pytest
import sys

# Try to import aditofpython
try:
    import aditofpython as tof
except ImportError as e:
    pytest.skip(f"aditofpython not available: {e}", allow_module_level=True)


@pytest.fixture
def system():
    """Fixture that provides an initialized System instance."""
    return tof.System()


@pytest.fixture
def camera_list(system):
    """Fixture that attempts to get available cameras from the system."""
    cameras = []
    try:
        system.getCameraList(cameras)
    except Exception as e:
        pytest.skip(f"Could not get camera list: {e}")
    return cameras


@pytest.fixture
def camera(camera_list):
    """Fixture that provides the first available camera."""
    if not camera_list:
        pytest.skip("No cameras available for testing")
    return camera_list[0]


@pytest.fixture
def initialized_camera(camera):
    """Fixture that provides an initialized camera."""
    try:
        status = camera.initialize()
        if status != tof.Status.OK:
            pytest.skip(f"Could not initialize camera: {status}")
    except Exception as e:
        pytest.skip(f"Camera initialization failed: {e}")
    return camera


@pytest.fixture
def frame():
    """Fixture that provides a Frame instance."""
    return tof.Frame()
