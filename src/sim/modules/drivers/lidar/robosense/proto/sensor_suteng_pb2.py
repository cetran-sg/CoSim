# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/drivers/lidar/robosense/proto/sensor_suteng.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common_msgs.basic_msgs import header_pb2 as modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/drivers/lidar/robosense/proto/sensor_suteng.proto',
  package='apollo.drivers.suteng',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n9modules/drivers/lidar/robosense/proto/sensor_suteng.proto\x12\x15\x61pollo.drivers.suteng\x1a+modules/common_msgs/basic_msgs/header.proto\"+\n\x0cSutengPacket\x12\r\n\x05stamp\x18\x01 \x01(\x04\x12\x0c\n\x04\x64\x61ta\x18\x02 \x02(\x0c\"\xba\x02\n\nSutengScan\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12+\n\x05model\x18\x02 \x02(\x0e\x32\x1c.apollo.drivers.suteng.Model\x12)\n\x04mode\x18\x03 \x02(\x0e\x32\x1b.apollo.drivers.suteng.Mode\x12\x38\n\x0b\x66iring_pkts\x18\x04 \x03(\x0b\x32#.apollo.drivers.suteng.SutengPacket\x12=\n\x10positioning_pkts\x18\x05 \x03(\x0b\x32#.apollo.drivers.suteng.SutengPacket\x12\n\n\x02sn\x18\x06 \x01(\t\x12\x13\n\x08\x62\x61setime\x18\x07 \x02(\x04:\x01\x30\x12\x13\n\x0btemperature\x18\x08 \x01(\x02*i\n\x05Model\x12\n\n\x06UNKOWN\x10\x00\x12\x0e\n\nHDL64E_S3S\x10\x01\x12\x0e\n\nHDL64E_S3D\x10\x02\x12\r\n\tHDL64E_S2\x10\x03\x12\n\n\x06HDL32E\x10\x04\x12\t\n\x05VLP16\x10\x05\x12\x0e\n\nHELIOS_16P\x10\x06*)\n\x04Mode\x12\r\n\tSTRONGEST\x10\x01\x12\x08\n\x04LAST\x10\x02\x12\x08\n\x04\x44UAL\x10\x03'
  ,
  dependencies=[modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2.DESCRIPTOR,])

_MODEL = _descriptor.EnumDescriptor(
  name='Model',
  full_name='apollo.drivers.suteng.Model',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKOWN', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='HDL64E_S3S', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='HDL64E_S3D', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='HDL64E_S2', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='HDL32E', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='VLP16', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='HELIOS_16P', index=6, number=6,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=491,
  serialized_end=596,
)
_sym_db.RegisterEnumDescriptor(_MODEL)

Model = enum_type_wrapper.EnumTypeWrapper(_MODEL)
_MODE = _descriptor.EnumDescriptor(
  name='Mode',
  full_name='apollo.drivers.suteng.Mode',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='STRONGEST', index=0, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='LAST', index=1, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DUAL', index=2, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=598,
  serialized_end=639,
)
_sym_db.RegisterEnumDescriptor(_MODE)

Mode = enum_type_wrapper.EnumTypeWrapper(_MODE)
UNKOWN = 0
HDL64E_S3S = 1
HDL64E_S3D = 2
HDL64E_S2 = 3
HDL32E = 4
VLP16 = 5
HELIOS_16P = 6
STRONGEST = 1
LAST = 2
DUAL = 3



_SUTENGPACKET = _descriptor.Descriptor(
  name='SutengPacket',
  full_name='apollo.drivers.suteng.SutengPacket',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='stamp', full_name='apollo.drivers.suteng.SutengPacket.stamp', index=0,
      number=1, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='data', full_name='apollo.drivers.suteng.SutengPacket.data', index=1,
      number=2, type=12, cpp_type=9, label=2,
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
  serialized_start=129,
  serialized_end=172,
)


_SUTENGSCAN = _descriptor.Descriptor(
  name='SutengScan',
  full_name='apollo.drivers.suteng.SutengScan',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.drivers.suteng.SutengScan.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='model', full_name='apollo.drivers.suteng.SutengScan.model', index=1,
      number=2, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='mode', full_name='apollo.drivers.suteng.SutengScan.mode', index=2,
      number=3, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='firing_pkts', full_name='apollo.drivers.suteng.SutengScan.firing_pkts', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='positioning_pkts', full_name='apollo.drivers.suteng.SutengScan.positioning_pkts', index=4,
      number=5, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='sn', full_name='apollo.drivers.suteng.SutengScan.sn', index=5,
      number=6, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='basetime', full_name='apollo.drivers.suteng.SutengScan.basetime', index=6,
      number=7, type=4, cpp_type=4, label=2,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='temperature', full_name='apollo.drivers.suteng.SutengScan.temperature', index=7,
      number=8, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
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
  serialized_start=175,
  serialized_end=489,
)

_SUTENGSCAN.fields_by_name['header'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2._HEADER
_SUTENGSCAN.fields_by_name['model'].enum_type = _MODEL
_SUTENGSCAN.fields_by_name['mode'].enum_type = _MODE
_SUTENGSCAN.fields_by_name['firing_pkts'].message_type = _SUTENGPACKET
_SUTENGSCAN.fields_by_name['positioning_pkts'].message_type = _SUTENGPACKET
DESCRIPTOR.message_types_by_name['SutengPacket'] = _SUTENGPACKET
DESCRIPTOR.message_types_by_name['SutengScan'] = _SUTENGSCAN
DESCRIPTOR.enum_types_by_name['Model'] = _MODEL
DESCRIPTOR.enum_types_by_name['Mode'] = _MODE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SutengPacket = _reflection.GeneratedProtocolMessageType('SutengPacket', (_message.Message,), {
  'DESCRIPTOR' : _SUTENGPACKET,
  '__module__' : 'modules.drivers.lidar.robosense.proto.sensor_suteng_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.suteng.SutengPacket)
  })
_sym_db.RegisterMessage(SutengPacket)

SutengScan = _reflection.GeneratedProtocolMessageType('SutengScan', (_message.Message,), {
  'DESCRIPTOR' : _SUTENGSCAN,
  '__module__' : 'modules.drivers.lidar.robosense.proto.sensor_suteng_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.suteng.SutengScan)
  })
_sym_db.RegisterMessage(SutengScan)


# @@protoc_insertion_point(module_scope)
