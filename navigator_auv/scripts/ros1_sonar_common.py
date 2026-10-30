"""Shared sonar decoding for the ros1_* ports of the original EROAS nodes."""

import numpy as np


def sonar_image_uint8(msg, float_gain):
    """Flat UINT8 intensities of a ProjectedSonarImage, on the scale the ROS 1
    EROAS thresholds were tuned to.

    The ROS 1 NPS sonar published UINT8 sensorGain*|P|. DAVE publishes FLOAT32
    20*log10|P|, so FLOAT32 frames are mapped back with float_gain playing the
    role of sensorGain. UINT8 frames are returned unchanged.

    The EROAS code takes beam 0 as the vehicle's right (polar_to_cartesian puts
    it at 45 deg, publish_heading turns left toward high beams). DAVE's beam 0
    is the vehicle's left (checked in World A: an obstacle only on the left
    lit beams 0-256), so FLOAT32 frames are also mirrored across the beams.
    """
    if msg.image.dtype == msg.image.DTYPE_FLOAT32:
        power_db = np.frombuffer(bytes(msg.image.data), dtype=np.float32)
        amplitude = np.power(10.0, np.nan_to_num(power_db, nan=-np.inf) / 20.0)
        u8 = np.clip(float_gain * amplitude, 0, 255).astype(np.uint8)
        beam_count = msg.image.beam_count or len(msg.beam_directions)
        return np.ascontiguousarray(u8.reshape(-1, beam_count)[:, ::-1]).ravel()
    return np.frombuffer(bytes(msg.image.data), dtype=np.uint8)
