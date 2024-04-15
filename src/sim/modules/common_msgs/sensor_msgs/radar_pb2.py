# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/common_msgs/sensor_msgs/radar.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common_msgs.basic_msgs import error_code_pb2 as modules_dot_common__msgs_dot_basic__msgs_dot_error__code__pb2
from modules.common_msgs.basic_msgs import geometry_pb2 as modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2
from modules.common_msgs.basic_msgs import header_pb2 as modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/common_msgs/sensor_msgs/radar.proto',
  package='apollo.drivers',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n+modules/common_msgs/sensor_msgs/radar.proto\x12\x0e\x61pollo.drivers\x1a/modules/common_msgs/basic_msgs/error_code.proto\x1a-modules/common_msgs/basic_msgs/geometry.proto\x1a+modules/common_msgs/basic_msgs/header.proto\"\xcc\x05\n\rRadarObstacle\x12\n\n\x02id\x18\x01 \x01(\x05\x12\x31\n\x11relative_position\x18\x02 \x01(\x0b\x32\x16.apollo.common.Point2D\x12\x31\n\x11relative_velocity\x18\x03 \x01(\x0b\x32\x16.apollo.common.Point2D\x12\x0b\n\x03rcs\x18\x04 \x01(\x01\x12\x41\n\rmoving_status\x18\x05 \x01(\x0e\x32*.apollo.drivers.RadarObstacle.MovingStatus\x12\r\n\x05width\x18\x06 \x01(\x01\x12\x0e\n\x06length\x18\x07 \x01(\x01\x12\x0e\n\x06height\x18\x08 \x01(\x01\x12\r\n\x05theta\x18\t \x01(\x01\x12\x31\n\x11\x61\x62solute_position\x18\n \x01(\x0b\x32\x16.apollo.common.Point2D\x12\x31\n\x11\x61\x62solute_velocity\x18\x0b \x01(\x0b\x32\x16.apollo.common.Point2D\x12\r\n\x05\x63ount\x18\x0c \x01(\x05\x12\x1b\n\x13moving_frames_count\x18\r \x01(\x05\x12\x34\n\x06status\x18\x0e \x01(\x0e\x32$.apollo.drivers.RadarObstacle.Status\"\xae\x01\n\x06Status\x12\r\n\tNO_TARGET\x10\x00\x12\x0e\n\nNEW_TARGET\x10\x01\x12\x16\n\x12NEW_UPDATED_TARGET\x10\x02\x12\x12\n\x0eUPDATED_TARGET\x10\x03\x12\x12\n\x0e\x43OASTED_TARGET\x10\x04\x12\x11\n\rMERGED_TARGET\x10\x05\x12\x1a\n\x16INVALID_COASTED_TARGET\x10\x06\x12\x16\n\x12NEW_COASTED_TARGET\x10\x07\"B\n\x0cMovingStatus\x12\x0e\n\nSTATIONARY\x10\x00\x12\x0b\n\x07NEARING\x10\x01\x12\x0b\n\x07\x41WAYING\x10\x02\x12\x08\n\x04NONE\x10\x03\"\x89\x02\n\x0eRadarObstacles\x12I\n\x0eradar_obstacle\x18\x01 \x03(\x0b\x32\x31.apollo.drivers.RadarObstacles.RadarObstacleEntry\x12%\n\x06header\x18\x02 \x01(\x0b\x32\x15.apollo.common.Header\x12\x30\n\nerror_code\x18\x03 \x01(\x0e\x32\x18.apollo.common.ErrorCode:\x02OK\x1aS\n\x12RadarObstacleEntry\x12\x0b\n\x03key\x18\x01 \x01(\x05\x12,\n\x05value\x18\x02 \x01(\x0b\x32\x1d.apollo.drivers.RadarObstacle:\x02\x38\x01'
  ,
  dependencies=[modules_dot_common__msgs_dot_basic__msgs_dot_error__code__pb2.DESCRIPTOR,modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2.DESCRIPTOR,modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2.DESCRIPTOR,])



_RADAROBSTACLE_STATUS = _descriptor.EnumDescriptor(
  name='Status',
  full_name='apollo.drivers.RadarObstacle.Status',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NO_TARGET', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='NEW_TARGET', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='NEW_UPDATED_TARGET', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='UPDATED_TARGET', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='COASTED_TARGET', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MERGED_TARGET', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='INVALID_COASTED_TARGET', index=6, number=6,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='NEW_COASTED_TARGET', index=7, number=7,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=679,
  serialized_end=853,
)
_sym_db.RegisterEnumDescriptor(_RADAROBSTACLE_STATUS)

_RADAROBSTACLE_MOVINGSTATUS = _descriptor.EnumDescriptor(
  name='MovingStatus',
  full_name='apollo.drivers.RadarObstacle.MovingStatus',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='STATIONARY', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='NEARING', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='AWAYING', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='NONE', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=855,
  serialized_end=921,
)
_sym_db.RegisterEnumDescriptor(_RADAROBSTACLE_MOVINGSTATUS)


_RADAROBSTACLE = _descriptor.Descriptor(
  name='RadarObstacle',
  full_name='apollo.drivers.RadarObstacle',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='apollo.drivers.RadarObstacle.id', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='relative_position', full_name='apollo.drivers.RadarObstacle.relative_position', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='relative_velocity', full_name='apollo.drivers.RadarObstacle.relative_velocity', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='rcs', full_name='apollo.drivers.RadarObstacle.rcs', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='moving_status', full_name='apollo.drivers.RadarObstacle.moving_status', index=4,
      number=5, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='width', full_name='apollo.drivers.RadarObstacle.width', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='length', full_name='apollo.drivers.RadarObstacle.length', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='height', full_name='apollo.drivers.RadarObstacle.height', index=7,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='theta', full_name='apollo.drivers.RadarObstacle.theta', index=8,
      number=9, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='absolute_position', full_name='apollo.drivers.RadarObstacle.absolute_position', index=9,
      number=10, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='absolute_velocity', full_name='apollo.drivers.RadarObstacle.absolute_velocity', index=10,
      number=11, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='count', full_name='apollo.drivers.RadarObstacle.count', index=11,
      number=12, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='moving_frames_count', full_name='apollo.drivers.RadarObstacle.moving_frames_count', index=12,
      number=13, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='status', full_name='apollo.drivers.RadarObstacle.status', index=13,
      number=14, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _RADAROBSTACLE_STATUS,
    _RADAROBSTACLE_MOVINGSTATUS,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=205,
  serialized_end=921,
)


_RADAROBSTACLES_RADAROBSTACLEENTRY = _descriptor.Descriptor(
  name='RadarObstacleEntry',
  full_name='apollo.drivers.RadarObstacles.RadarObstacleEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.drivers.RadarObstacles.RadarObstacleEntry.key', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.drivers.RadarObstacles.RadarObstacleEntry.value', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=b'8\001',
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1106,
  serialized_end=1189,
)

_RADAROBSTACLES = _descriptor.Descriptor(
  name='RadarObstacles',
  full_name='apollo.drivers.RadarObstacles',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='radar_obstacle', full_name='apollo.drivers.RadarObstacles.radar_obstacle', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.drivers.RadarObstacles.header', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='error_code', full_name='apollo.drivers.RadarObstacles.error_code', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_RADAROBSTACLES_RADAROBSTACLEENTRY, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=924,
  serialized_end=1189,
)

_RADAROBSTACLE.fields_by_name['relative_position'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2._POINT2D
_RADAROBSTACLE.fields_by_name['relative_velocity'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2._POINT2D
_RADAROBSTACLE.fields_by_name['moving_status'].enum_type = _RADAROBSTACLE_MOVINGSTATUS
_RADAROBSTACLE.fields_by_name['absolute_position'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2._POINT2D
_RADAROBSTACLE.fields_by_name['absolute_velocity'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_geometry__pb2._POINT2D
_RADAROBSTACLE.fields_by_name['status'].enum_type = _RADAROBSTACLE_STATUS
_RADAROBSTACLE_STATUS.containing_type = _RADAROBSTACLE
_RADAROBSTACLE_MOVINGSTATUS.containing_type = _RADAROBSTACLE
_RADAROBSTACLES_RADAROBSTACLEENTRY.fields_by_name['value'].message_type = _RADAROBSTACLE
_RADAROBSTACLES_RADAROBSTACLEENTRY.containing_type = _RADAROBSTACLES
_RADAROBSTACLES.fields_by_name['radar_obstacle'].message_type = _RADAROBSTACLES_RADAROBSTACLEENTRY
_RADAROBSTACLES.fields_by_name['header'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2._HEADER
_RADAROBSTACLES.fields_by_name['error_code'].enum_type = modules_dot_common__msgs_dot_basic__msgs_dot_error__code__pb2._ERRORCODE
DESCRIPTOR.message_types_by_name['RadarObstacle'] = _RADAROBSTACLE
DESCRIPTOR.message_types_by_name['RadarObstacles'] = _RADAROBSTACLES
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

RadarObstacle = _reflection.GeneratedProtocolMessageType('RadarObstacle', (_message.Message,), {
  'DESCRIPTOR' : _RADAROBSTACLE,
  '__module__' : 'modules.common_msgs.sensor_msgs.radar_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.RadarObstacle)
  })
_sym_db.RegisterMessage(RadarObstacle)

RadarObstacles = _reflection.GeneratedProtocolMessageType('RadarObstacles', (_message.Message,), {

  'RadarObstacleEntry' : _reflection.GeneratedProtocolMessageType('RadarObstacleEntry', (_message.Message,), {
    'DESCRIPTOR' : _RADAROBSTACLES_RADAROBSTACLEENTRY,
    '__module__' : 'modules.common_msgs.sensor_msgs.radar_pb2'
    # @@protoc_insertion_point(class_scope:apollo.drivers.RadarObstacles.RadarObstacleEntry)
    })
  ,
  'DESCRIPTOR' : _RADAROBSTACLES,
  '__module__' : 'modules.common_msgs.sensor_msgs.radar_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.RadarObstacles)
  })
_sym_db.RegisterMessage(RadarObstacles)
_sym_db.RegisterMessage(RadarObstacles.RadarObstacleEntry)


_RADAROBSTACLES_RADAROBSTACLEENTRY._options = None
# @@protoc_insertion_point(module_scope)
