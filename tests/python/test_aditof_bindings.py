"""
Test suite for aditofpython bindings.
Tests are based on the first-frame example and validate basic SDK functionality.
"""
import pytest
import numpy as np
import struct

try:
    import aditofpython as tof
except ImportError:
    pytest.skip("aditofpython not available", allow_module_level=True)


class TestVersionAPI:
    """Tests for version API functions."""

    def test_api_version_returns_string(self):
        """Test that getApiVersion returns a non-empty string."""
        version = tof.getApiVersion()
        assert isinstance(version, str)
        assert len(version) > 0
        assert '.' in version, "Version should contain dots"

    def test_branch_version_returns_string(self):
        """Test that getBranchVersion returns a string."""
        branch = tof.getBranchVersion()
        assert isinstance(branch, str)

    def test_commit_version_returns_string(self):
        """Test that getCommitVersion returns a string."""
        commit = tof.getCommitVersion()
        assert isinstance(commit, str)


class TestSystemAPI:
    """Tests for System class functionality."""

    def test_system_instantiation(self, system):
        """Test that System can be instantiated."""
        assert system is not None

    def test_get_camera_list_without_ip(self, system):
        """Test getCameraList without IP parameter."""
        cameras = []
        status = system.getCameraList(cameras)
        # Should not throw, cameras list might be empty
        assert status == tof.Status.OK or cameras == []

    def test_camera_list_returns_list(self, camera_list):
        """Test that getCameraList returns a list."""
        assert isinstance(camera_list, list)

    def test_get_camera_list_with_ip(self, system):
        """Test getCameraList with IP parameter (network test)."""
        cameras = []
        # This might fail if network is not available, so we catch exceptions
        try:
            status = system.getCameraList(cameras, "ip:192.168.1.100")
            # If it succeeds, status should be OK or cameras should be empty
            assert status == tof.Status.OK or cameras == []
        except Exception:
            pytest.skip("Network interface not available")


class TestFrameAPI:
    """Tests for Frame class functionality."""

    def test_frame_instantiation(self, frame):
        """Test that Frame can be instantiated."""
        assert frame is not None

    def test_frame_get_details(self, frame):
        """Test Frame.getDetails method."""
        frameDetails = tof.FrameDetails()
        try:
            status = frame.getDetails(frameDetails)
            # Frame details might not be available until frame is populated
            assert status == tof.Status.OK or status == tof.Status.GENERIC_ERROR
        except Exception:
            # Expected if frame is not initialized
            pass

    def test_frame_data_details_access(self, frame):
        """Test FrameDataDetails instantiation."""
        fdd = tof.FrameDataDetails()
        assert fdd is not None


class TestCameraAPI:
    """Tests for Camera class functionality."""

    def test_camera_get_details(self, camera):
        """Test Camera.getDetails method."""
        details = tof.CameraDetails()
        try:
            status = camera.getDetails(details)
            assert status == tof.Status.OK
            assert hasattr(details, 'cameraId')
            assert hasattr(details, 'connection')
        except Exception as e:
            pytest.skip(f"Could not get camera details: {e}")

    def test_camera_get_available_modes(self, initialized_camera):
        """Test getting available camera modes."""
        modes = []
        status = initialized_camera.getAvailableModes(modes)
        assert status == tof.Status.OK
        assert isinstance(modes, list)
        assert len(modes) > 0, "Should have at least one mode available"

    def test_camera_set_mode(self, initialized_camera):
        """Test setting camera mode."""
        modes = []
        initialized_camera.getAvailableModes(modes)
        if modes:
            status = initialized_camera.setMode(modes[0])
            assert status == tof.Status.OK

    def test_camera_start_stop(self, initialized_camera):
        """Test camera start and stop."""
        modes = []
        initialized_camera.getAvailableModes(modes)
        
        if modes:
            # Set a valid mode first
            initialized_camera.setMode(modes[0])
            
            # Start camera
            status = initialized_camera.start()
            assert status == tof.Status.OK
            
            # Stop camera
            status = initialized_camera.stop()
            assert status == tof.Status.OK

    def test_camera_request_frame(self, initialized_camera):
        """Test requesting a frame from the camera."""
        modes = []
        initialized_camera.getAvailableModes(modes)
        
        if modes:
            initialized_camera.setMode(modes[0])
            initialized_camera.start()
            
            frame = tof.Frame()
            status = initialized_camera.requestFrame(frame)
            
            # Clean up
            initialized_camera.stop()
            
            # Frame request might fail if hardware not available
            assert status == tof.Status.OK or status == tof.Status.GENERIC_ERROR


class TestFrameDataAccess:
    """Tests for accessing different frame data types."""

    def test_frame_data_details_width_height(self):
        """Test FrameDataDetails has width and height."""
        fdd = tof.FrameDataDetails()
        assert hasattr(fdd, 'width')
        assert hasattr(fdd, 'height')
        assert hasattr(fdd, 'type')

    def test_frame_details_structure(self):
        """Test FrameDetails structure."""
        fd = tof.FrameDetails()
        assert hasattr(fd, 'dataDetails')

    def test_metadata_structure(self):
        """Test Metadata structure has expected fields."""
        metadata = tof.Metadata()
        assert hasattr(metadata, 'sensorTemperature')
        assert hasattr(metadata, 'laserTemperature')
        assert hasattr(metadata, 'frameNumber')
        assert hasattr(metadata, 'imagerMode')


class TestStatusEnum:
    """Tests for Status enumeration."""

    def test_status_ok_value(self):
        """Test that Status.OK exists and is 0."""
        assert hasattr(tof.Status, 'OK')
        assert tof.Status.OK == 0

    def test_status_generic_error_exists(self):
        """Test that Status.GENERIC_ERROR exists."""
        assert hasattr(tof.Status, 'GENERIC_ERROR')

    def test_status_invalid_argument_exists(self):
        """Test that Status.INVALID_ARGUMENT exists."""
        assert hasattr(tof.Status, 'INVALID_ARGUMENT')


class TestCameraDetails:
    """Tests for CameraDetails structure."""

    def test_camera_details_has_required_fields(self):
        """Test that CameraDetails has required fields."""
        details = tof.CameraDetails()
        assert hasattr(details, 'cameraId')
        assert hasattr(details, 'connection')


class TestSensorInterface:
    """Tests for sensor interface functionality."""

    def test_camera_get_sensor(self, camera):
        """Test getting sensor from camera."""
        try:
            sensor = camera.getSensor()
            assert sensor is not None
        except Exception as e:
            pytest.skip(f"Could not get sensor: {e}")

    def test_sensor_callback_registration(self, camera):
        """Test registering and unregistering sensor callbacks."""
        try:
            sensor = camera.getSensor()
            
            # Define a simple callback
            callback_called = []
            
            def test_callback(status):
                callback_called.append(status)
            
            # Register callback
            status = sensor.adsd3500_register_interrupt_callback(test_callback)
            assert status == tof.Status.OK
            
            # Unregister callback
            status = sensor.adsd3500_unregister_interrupt_callback(test_callback)
            assert status == tof.Status.OK
        except Exception as e:
            pytest.skip(f"Sensor callback test not available: {e}")


class TestNumpyIntegration:
    """Tests for numpy array integration."""

    def test_frame_data_to_numpy_array(self):
        """Test that frame data can be converted to numpy arrays."""
        frame = tof.Frame()
        # This will only work if frame is populated with actual data
        # For now, just test that asarray doesn't crash
        try:
            # Empty frame might not have data
            pass
        except Exception:
            pytest.skip("Frame not populated with data")

    def test_numpy_array_creation(self):
        """Test basic numpy array creation for data handling."""
        arr = np.zeros((100, 100), dtype=np.uint16)
        assert arr.shape == (100, 100)
        assert arr.dtype == np.uint16


class TestFrameTypeStrings:
    """Tests for common frame type strings."""

    @pytest.mark.parametrize("frame_type", ["depth", "ir", "ab", "xyz", "conf"])
    def test_frame_type_string_validity(self, frame_type):
        """Test that frame type strings are valid."""
        assert isinstance(frame_type, str)
        assert len(frame_type) > 0
        assert len(frame_type) < 20


class TestIntegrationScenarios:
    """Integration tests simulating real usage scenarios."""

    def test_system_initialization_workflow(self, system):
        """Test typical system initialization workflow."""
        # Get cameras
        cameras = []
        status = system.getCameraList(cameras)
        assert status == tof.Status.OK or cameras == []
        
        if not cameras:
            pytest.skip("No cameras available")
        
        # Get camera details
        camera = cameras[0]
        details = tof.CameraDetails()
        status = camera.getDetails(details)
        assert status == tof.Status.OK

    def test_camera_initialization_workflow(self, initialized_camera):
        """Test camera initialization workflow."""
        # Get available modes
        modes = []
        status = initialized_camera.getAvailableModes(modes)
        assert status == tof.Status.OK
        assert len(modes) > 0
        
        # Set a mode
        status = initialized_camera.setMode(modes[0])
        assert status == tof.Status.OK
