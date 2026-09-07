"""Project-specific exceptions. Never swallow these silently."""


class ViveTrackingError(Exception):
    """Base error for the VIVE tracking module."""


class ConfigError(ViveTrackingError):
    """Invalid or missing configuration."""


class BackendNotAvailableError(ViveTrackingError):
    """Requested backend cannot be loaded on this machine."""


class BackendInitError(ViveTrackingError):
    """Backend is present but failed to initialize."""


class DeviceNotFoundError(ViveTrackingError):
    """Requested tracker serial or logical_name is not known."""
