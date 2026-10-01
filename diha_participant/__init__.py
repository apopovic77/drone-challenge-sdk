"""Small participant-side logger and live uploader for DRONE-Challenge 1A.

The package records the team's own estimate. Ground truth is intentionally not
accepted by :class:`SessionLogger` in competition mode; the hall reference is sent
by the organiser's separate forwarder with its own session token.
"""

from .client import ApiClient, ApiError
from .live import DroneLive
from .logger import LIVE_EVENT_KINDS, Event, Pose, SessionLogger
from .uploader import TelemetryUploader, UploadStats

__all__ = [
    "LIVE_EVENT_KINDS",
    "ApiClient",
    "ApiError",
    "DroneLive",
    "Event",
    "Pose",
    "SessionLogger",
    "TelemetryUploader",
    "UploadStats",
]
