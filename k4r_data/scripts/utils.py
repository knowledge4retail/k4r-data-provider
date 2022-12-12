from tf.transformations import \
    quaternion_from_euler


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
        orientation = quaternion_from_euler(roll, pitch, yaw)
    assert len(orientation) == 4,\
        f'orientation size of {orientation}'\
        f' does not match 4 for {self.uri}'
    return orientation
