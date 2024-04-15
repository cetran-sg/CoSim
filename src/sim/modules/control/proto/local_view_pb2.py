# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/control/proto/local_view.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common_msgs.chassis_msgs import chassis_pb2 as modules_dot_common__msgs_dot_chassis__msgs_dot_chassis__pb2
from modules.common_msgs.basic_msgs import header_pb2 as modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2
from modules.common_msgs.control_msgs import pad_msg_pb2 as modules_dot_common__msgs_dot_control__msgs_dot_pad__msg__pb2
from modules.common_msgs.localization_msgs import localization_pb2 as modules_dot_common__msgs_dot_localization__msgs_dot_localization__pb2
from modules.common_msgs.planning_msgs import planning_pb2 as modules_dot_common__msgs_dot_planning__msgs_dot_planning__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/control/proto/local_view.proto',
  package='apollo.control',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n&modules/control/proto/local_view.proto\x12\x0e\x61pollo.control\x1a.modules/common_msgs/chassis_msgs/chassis.proto\x1a+modules/common_msgs/basic_msgs/header.proto\x1a.modules/common_msgs/control_msgs/pad_msg.proto\x1a\x38modules/common_msgs/localization_msgs/localization.proto\x1a\x30modules/common_msgs/planning_msgs/planning.proto\"\xfd\x01\n\tLocalView\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12\'\n\x07\x63hassis\x18\x02 \x01(\x0b\x32\x16.apollo.canbus.Chassis\x12\x32\n\ntrajectory\x18\x03 \x01(\x0b\x32\x1e.apollo.planning.ADCTrajectory\x12?\n\x0clocalization\x18\x04 \x01(\x0b\x32).apollo.localization.LocalizationEstimate\x12+\n\x07pad_msg\x18\x05 \x01(\x0b\x32\x1a.apollo.control.PadMessage'
  ,
  dependencies=[modules_dot_common__msgs_dot_chassis__msgs_dot_chassis__pb2.DESCRIPTOR,modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2.DESCRIPTOR,modules_dot_common__msgs_dot_control__msgs_dot_pad__msg__pb2.DESCRIPTOR,modules_dot_common__msgs_dot_localization__msgs_dot_localization__pb2.DESCRIPTOR,modules_dot_common__msgs_dot_planning__msgs_dot_planning__pb2.DESCRIPTOR,])




_LOCALVIEW = _descriptor.Descriptor(
  name='LocalView',
  full_name='apollo.control.LocalView',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.control.LocalView.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='chassis', full_name='apollo.control.LocalView.chassis', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='trajectory', full_name='apollo.control.LocalView.trajectory', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='localization', full_name='apollo.control.LocalView.localization', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pad_msg', full_name='apollo.control.LocalView.pad_msg', index=4,
      number=5, type=11, cpp_type=10, label=1,
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
  serialized_start=308,
  serialized_end=561,
)

_LOCALVIEW.fields_by_name['header'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2._HEADER
_LOCALVIEW.fields_by_name['chassis'].message_type = modules_dot_common__msgs_dot_chassis__msgs_dot_chassis__pb2._CHASSIS
_LOCALVIEW.fields_by_name['trajectory'].message_type = modules_dot_common__msgs_dot_planning__msgs_dot_planning__pb2._ADCTRAJECTORY
_LOCALVIEW.fields_by_name['localization'].message_type = modules_dot_common__msgs_dot_localization__msgs_dot_localization__pb2._LOCALIZATIONESTIMATE
_LOCALVIEW.fields_by_name['pad_msg'].message_type = modules_dot_common__msgs_dot_control__msgs_dot_pad__msg__pb2._PADMESSAGE
DESCRIPTOR.message_types_by_name['LocalView'] = _LOCALVIEW
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LocalView = _reflection.GeneratedProtocolMessageType('LocalView', (_message.Message,), {
  'DESCRIPTOR' : _LOCALVIEW,
  '__module__' : 'modules.control.proto.local_view_pb2'
  # @@protoc_insertion_point(class_scope:apollo.control.LocalView)
  })
_sym_db.RegisterMessage(LocalView)


# @@protoc_insertion_point(module_scope)