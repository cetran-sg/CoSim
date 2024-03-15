# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/drivers/lidar/proto/robosense.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common_msgs.basic_msgs import header_pb2 as modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/drivers/lidar/proto/robosense.proto',
  package='apollo.drivers.robosense',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n+modules/drivers/lidar/proto/robosense.proto\x12\x18\x61pollo.drivers.robosense\x1a+modules/common_msgs/basic_msgs/header.proto\"2\n\x13RobosenseScanPacket\x12\r\n\x05stamp\x18\x01 \x01(\x04\x12\x0c\n\x04\x64\x61ta\x18\x02 \x01(\x0c\"\x9e\x01\n\rRobosenseScan\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12\r\n\x05model\x18\x02 \x01(\t\x12\x42\n\x0b\x66iring_pkts\x18\x03 \x03(\x0b\x32-.apollo.drivers.robosense.RobosenseScanPacket\x12\x13\n\x08\x62\x61setime\x18\x04 \x01(\x04:\x01\x30'
  ,
  dependencies=[modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2.DESCRIPTOR,])




_ROBOSENSESCANPACKET = _descriptor.Descriptor(
  name='RobosenseScanPacket',
  full_name='apollo.drivers.robosense.RobosenseScanPacket',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='stamp', full_name='apollo.drivers.robosense.RobosenseScanPacket.stamp', index=0,
      number=1, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='data', full_name='apollo.drivers.robosense.RobosenseScanPacket.data', index=1,
      number=2, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=b"",
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
  serialized_start=118,
  serialized_end=168,
)


_ROBOSENSESCAN = _descriptor.Descriptor(
  name='RobosenseScan',
  full_name='apollo.drivers.robosense.RobosenseScan',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.drivers.robosense.RobosenseScan.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='model', full_name='apollo.drivers.robosense.RobosenseScan.model', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='firing_pkts', full_name='apollo.drivers.robosense.RobosenseScan.firing_pkts', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='basetime', full_name='apollo.drivers.robosense.RobosenseScan.basetime', index=3,
      number=4, type=4, cpp_type=4, label=1,
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
  serialized_start=171,
  serialized_end=329,
)

_ROBOSENSESCAN.fields_by_name['header'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2._HEADER
_ROBOSENSESCAN.fields_by_name['firing_pkts'].message_type = _ROBOSENSESCANPACKET
DESCRIPTOR.message_types_by_name['RobosenseScanPacket'] = _ROBOSENSESCANPACKET
DESCRIPTOR.message_types_by_name['RobosenseScan'] = _ROBOSENSESCAN
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

RobosenseScanPacket = _reflection.GeneratedProtocolMessageType('RobosenseScanPacket', (_message.Message,), {
  'DESCRIPTOR' : _ROBOSENSESCANPACKET,
  '__module__' : 'modules.drivers.lidar.proto.robosense_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.robosense.RobosenseScanPacket)
  })
_sym_db.RegisterMessage(RobosenseScanPacket)

RobosenseScan = _reflection.GeneratedProtocolMessageType('RobosenseScan', (_message.Message,), {
  'DESCRIPTOR' : _ROBOSENSESCAN,
  '__module__' : 'modules.drivers.lidar.proto.robosense_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.robosense.RobosenseScan)
  })
_sym_db.RegisterMessage(RobosenseScan)


# @@protoc_insertion_point(module_scope)
