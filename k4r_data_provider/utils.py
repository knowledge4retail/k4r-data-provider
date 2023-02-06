import math
import dataclasses
import json


class DataClassToDictEncoder(json.JSONEncoder):
    # Helper class for JSON decoding of dataclasses
    def default(self, obj):
        if dataclasses.is_dataclass(obj):
            return dataclasses.asdict(obj)
        return super().default(data)


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> set:
    # roll (X), pitch (Y), yaw (Z),
    # Abbreviations for the various angular functions based on 
    # https://en.wikipedia.org/wiki/
    # Conversion_between_quaternions_and_Euler_angles
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return (
        sr * cp * cy - cr * sp * sy, # x
        cr * sp * cy + sr * cp * sy, # y
        cr * cp * sy - sr * sp * cy, # z
        cr * cp * cy + sr * sp * sy  # w
    )


def quaternion_to_euler(x: float, y: float, z: float, w: float):
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    # math.asin has to be between -1 and 1, so we return 90°(π/2) as limit
    sinp = max(-1, sinp)
    sinp = min(1, sinp)
    pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def orientation_from_list(orientation, quaternion=True, radians=False):
    if not quaternion:
        roll = orientation[0]
        pitch = orientation[1]
        yaw = orientation[2]
        assert len(orientation) == 3,\
            f'orientation size of {orientation} '\
            f'does not match 3 for {self.uri}'
        if not radians:
            # convert degree to radians (default)
            degrees2rad = math.pi / 180.0
            roll *= degrees2rad
            pitch *= degrees2rad
            yaw *= degrees2rad
        orientation = euler_to_quaternion(roll, pitch, yaw)
    assert len(orientation) == 4,\
        f'orientation size of {orientation}'\
        f' does not match 4 for {self.uri}'
    return orientation
