from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class ImageRequest(_message.Message):
    __slots__ = ["ego_x", "ego_y", "ego_z", "ego_roll", "ego_pitch", "ego_yaw", "timestamp"]
    EGO_X_FIELD_NUMBER: _ClassVar[int]
    EGO_Y_FIELD_NUMBER: _ClassVar[int]
    EGO_Z_FIELD_NUMBER: _ClassVar[int]
    EGO_ROLL_FIELD_NUMBER: _ClassVar[int]
    EGO_PITCH_FIELD_NUMBER: _ClassVar[int]
    EGO_YAW_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    ego_x: float
    ego_y: float
    ego_z: float
    ego_roll: float
    ego_pitch: float
    ego_yaw: float
    timestamp: int
    def __init__(self, ego_x: _Optional[float] = ..., ego_y: _Optional[float] = ..., ego_z: _Optional[float] = ..., ego_roll: _Optional[float] = ..., ego_pitch: _Optional[float] = ..., ego_yaw: _Optional[float] = ..., timestamp: _Optional[int] = ...) -> None: ...

class ImageResponse(_message.Message):
    __slots__ = ["front_left_image", "front_right_image", "front_center_image", "left_front_image", "left_side_image", "left_rear_image", "right_front_image", "right_side_image", "right_rear_image"]
    FRONT_LEFT_IMAGE_FIELD_NUMBER: _ClassVar[int]
    FRONT_RIGHT_IMAGE_FIELD_NUMBER: _ClassVar[int]
    FRONT_CENTER_IMAGE_FIELD_NUMBER: _ClassVar[int]
    LEFT_FRONT_IMAGE_FIELD_NUMBER: _ClassVar[int]
    LEFT_SIDE_IMAGE_FIELD_NUMBER: _ClassVar[int]
    LEFT_REAR_IMAGE_FIELD_NUMBER: _ClassVar[int]
    RIGHT_FRONT_IMAGE_FIELD_NUMBER: _ClassVar[int]
    RIGHT_SIDE_IMAGE_FIELD_NUMBER: _ClassVar[int]
    RIGHT_REAR_IMAGE_FIELD_NUMBER: _ClassVar[int]
    front_left_image: bytes
    front_right_image: bytes
    front_center_image: bytes
    left_front_image: bytes
    left_side_image: bytes
    left_rear_image: bytes
    right_front_image: bytes
    right_side_image: bytes
    right_rear_image: bytes
    def __init__(self, front_left_image: _Optional[bytes] = ..., front_right_image: _Optional[bytes] = ..., front_center_image: _Optional[bytes] = ..., left_front_image: _Optional[bytes] = ..., left_side_image: _Optional[bytes] = ..., left_rear_image: _Optional[bytes] = ..., right_front_image: _Optional[bytes] = ..., right_side_image: _Optional[bytes] = ..., right_rear_image: _Optional[bytes] = ...) -> None: ...
