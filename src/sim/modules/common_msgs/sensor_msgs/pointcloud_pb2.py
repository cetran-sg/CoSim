# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/common_msgs/sensor_msgs/pointcloud.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common_msgs.basic_msgs import header_pb2 as modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/common_msgs/sensor_msgs/pointcloud.proto',
  package='apollo.drivers',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n0modules/common_msgs/sensor_msgs/pointcloud.proto\x12\x0e\x61pollo.drivers\x1a+modules/common_msgs/basic_msgs/header.proto\"h\n\nPointXYZIT\x12\x0e\n\x01x\x18\x01 \x01(\x02:\x03nan\x12\x0e\n\x01y\x18\x02 \x01(\x02:\x03nan\x12\x0e\n\x01z\x18\x03 \x01(\x02:\x03nan\x12\x14\n\tintensity\x18\x04 \x01(\r:\x01\x30\x12\x14\n\ttimestamp\x18\x05 \x01(\x04:\x01\x30\"\xbb\x01\n\nPointCloud\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12\x10\n\x08\x66rame_id\x18\x02 \x01(\t\x12\x10\n\x08is_dense\x18\x03 \x01(\x08\x12)\n\x05point\x18\x04 \x03(\x0b\x32\x1a.apollo.drivers.PointXYZIT\x12\x18\n\x10measurement_time\x18\x05 \x01(\x01\x12\r\n\x05width\x18\x06 \x01(\r\x12\x0e\n\x06height\x18\x07 \x01(\r'
  ,
  dependencies=[modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2.DESCRIPTOR,])




_POINTXYZIT = _descriptor.Descriptor(
  name='PointXYZIT',
  full_name='apollo.drivers.PointXYZIT',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='apollo.drivers.PointXYZIT.x', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=True, default_value=(1e10000 * 0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='y', full_name='apollo.drivers.PointXYZIT.y', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=True, default_value=(1e10000 * 0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='z', full_name='apollo.drivers.PointXYZIT.z', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=True, default_value=(1e10000 * 0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='intensity', full_name='apollo.drivers.PointXYZIT.intensity', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='apollo.drivers.PointXYZIT.timestamp', index=4,
      number=5, type=4, cpp_type=4, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=113,
  serialized_end=217,
)


_POINTCLOUD = _descriptor.Descriptor(
  name='PointCloud',
  full_name='apollo.drivers.PointCloud',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.drivers.PointCloud.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='frame_id', full_name='apollo.drivers.PointCloud.frame_id', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='is_dense', full_name='apollo.drivers.PointCloud.is_dense', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='point', full_name='apollo.drivers.PointCloud.point', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='measurement_time', full_name='apollo.drivers.PointCloud.measurement_time', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='width', full_name='apollo.drivers.PointCloud.width', index=5,
      number=6, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='height', full_name='apollo.drivers.PointCloud.height', index=6,
      number=7, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=220,
  serialized_end=407,
)

_POINTCLOUD.fields_by_name['header'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2._HEADER
_POINTCLOUD.fields_by_name['point'].message_type = _POINTXYZIT
DESCRIPTOR.message_types_by_name['PointXYZIT'] = _POINTXYZIT
DESCRIPTOR.message_types_by_name['PointCloud'] = _POINTCLOUD
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PointXYZIT = _reflection.GeneratedProtocolMessageType('PointXYZIT', (_message.Message,), {
  'DESCRIPTOR' : _POINTXYZIT,
  '__module__' : 'modules.common_msgs.sensor_msgs.pointcloud_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.PointXYZIT)
  })
_sym_db.RegisterMessage(PointXYZIT)

PointCloud = _reflection.GeneratedProtocolMessageType('PointCloud', (_message.Message,), {
  'DESCRIPTOR' : _POINTCLOUD,
  '__module__' : 'modules.common_msgs.sensor_msgs.pointcloud_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.PointCloud)
  })
_sym_db.RegisterMessage(PointCloud)


# @@protoc_insertion_point(module_scope)
