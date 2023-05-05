"""
dvrk_rcm_estimation package

This package provides tools for estimating the center of rotation for the Da Vinci PSM arm using NDI sensors.

Author: Jaspor Jiang
Email: sjiang44@jh.edu
Date: 2023-05-05
License: MIT

Modules:
    ndi_sensor - Contains the NDIsensor class for interacting with NDI sensors.
    residual_error - Provides a function to calculate the residual error for the center of rotation estimation.
    record_base_manual - Script for manually recording base positions and saving them to a file.
    record_base_auto - Script for automatically recording base positions and saving them to a file.
    compute_center - Script for computing the center of rotation from the recorded base positions.
"""

from .ndi_sensor import NDIsensor
from .residual_error import residual_error
from .record_base_manual import record_base_manual
from .record_base_auto import record_base_auto
from .compute_center import compute_center
