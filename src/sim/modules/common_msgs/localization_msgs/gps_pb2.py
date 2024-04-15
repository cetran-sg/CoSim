# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/common_msgs/localization_msgs/gps.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common_msgs.basic_msgs import header_pb2 as modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2
from modules.common_msgs.localization_msgs import pose_pb2 as modules_dot_common__msgs_dot_localization__msgs_dot_pose__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/common_msgs/localization_msgs/gps.proto',
  package='apollo.localization',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n/modules/common_msgs/localization_msgs/gps.proto\x12\x13\x61pollo.localization\x1a+modules/common_msgs/basic_msgs/header.proto\x1a\x30modules/common_msgs/localization_msgs/pose.proto\"]\n\x03Gps\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12/\n\x0clocalization\x18\x02 \x01(\x0b\x32\x19.apollo.localization.Pose'
  ,
  dependencies=[modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2.DESCRIPTOR,modules_dot_common__msgs_dot_localization__msgs_dot_pose__pb2.DESCRIPTOR,])




_GPS = _descriptor.Descriptor(
  name='Gps',
  full_name='apollo.localization.Gps',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.localization.Gps.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='localization', full_name='apollo.localization.Gps.localization', index=1,
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
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=167,
  serialized_end=260,
)

_GPS.fields_by_name['header'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2._HEADER
_GPS.fields_by_name['localization'].message_type = modules_dot_common__msgs_dot_localization__msgs_dot_pose__pb2._POSE
DESCRIPTOR.message_types_by_name['Gps'] = _GPS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Gps = _reflection.GeneratedProtocolMessageType('Gps', (_message.Message,), {
  'DESCRIPTOR' : _GPS,
  '__module__' : 'modules.common_msgs.localization_msgs.gps_pb2'
  # @@protoc_insertion_point(class_scope:apollo.localization.Gps)
  })
_sym_db.RegisterMessage(Gps)


# @@protoc_insertion_point(module_scope)
