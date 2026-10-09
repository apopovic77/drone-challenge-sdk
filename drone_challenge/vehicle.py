"""Vehicle size declarations for new sessions (metres, including propellers)."""

import math
import warnings


def validate_size(size_m):
    if size_m is None:
        return None
    if not isinstance(size_m, (tuple, list)) or len(size_m) != 3:
        raise ValueError("size_m must contain width, depth and height in metres")
    if any(
        isinstance(v, bool)
        or not isinstance(v, (int, float))
        or not math.isfinite(v)
        or not 0 < v <= 5
        for v in size_m
    ):
        raise ValueError("size_m dimensions must be finite numbers greater than 0 and at most 5 m")
    return tuple(float(v) for v in size_m)


def warn_profile(value):
    """Surface server decisions at creation, before any flight is started."""
    if not value:
        return
    if value.get("declaration_conflict"):
        warnings.warn(
            f"Declared vehicle size {value['declared_size_m']} m overridden by "
            f"organiser size {value['size_m']} m.",
            stacklevel=3,
        )
    if value.get("size_m") and not value.get("profile"):
        warnings.warn(
            "Course has no vehicle profile; size is recorded but not evaluated.", stacklevel=3
        )
    proof = value.get("route_clearance", {})
    if proof.get("status") == "insufficient":
        warnings.warn(
            f"Route clearance insufficient for this vehicle: minimum "
            f"{proof['minimum_m']:.4f} m, required {proof['required_m']:.4f} m "
            f"(shortfall {proof['shortfall_m']:.4f} m). Start remains allowed.",
            stacklevel=3,
        )
