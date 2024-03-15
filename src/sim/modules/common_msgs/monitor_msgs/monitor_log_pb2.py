# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/common_msgs/monitor_msgs/monitor_log.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common_msgs.basic_msgs import header_pb2 as modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/common_msgs/monitor_msgs/monitor_log.proto',
  package='apollo.common.monitor',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n2modules/common_msgs/monitor_msgs/monitor_log.proto\x12\x15\x61pollo.common.monitor\x1a+modules/common_msgs/basic_msgs/header.proto\"\xc1\x04\n\x12MonitorMessageItem\x12P\n\x06source\x18\x01 \x01(\x0e\x32\x37.apollo.common.monitor.MonitorMessageItem.MessageSource:\x07UNKNOWN\x12\x0b\n\x03msg\x18\x02 \x01(\t\x12K\n\tlog_level\x18\x03 \x01(\x0e\x32\x32.apollo.common.monitor.MonitorMessageItem.LogLevel:\x04INFO\"\xc8\x02\n\rMessageSource\x12\x0b\n\x07UNKNOWN\x10\x01\x12\n\n\x06\x43\x41NBUS\x10\x02\x12\x0b\n\x07\x43ONTROL\x10\x03\x12\x0c\n\x08\x44\x45\x43ISION\x10\x04\x12\x10\n\x0cLOCALIZATION\x10\x05\x12\x0c\n\x08PLANNING\x10\x06\x12\x0e\n\nPREDICTION\x10\x07\x12\r\n\tSIMULATOR\x10\x08\x12\t\n\x05HWSYS\x10\t\x12\x0b\n\x07ROUTING\x10\n\x12\x0b\n\x07MONITOR\x10\x0b\x12\x07\n\x03HMI\x10\x0c\x12\x10\n\x0cRELATIVE_MAP\x10\r\x12\x08\n\x04GNSS\x10\x0e\x12\x0f\n\x0b\x43ONTI_RADAR\x10\x0f\x12\x11\n\rRACOBIT_RADAR\x10\x10\x12\x14\n\x10ULTRASONIC_RADAR\x10\x11\x12\x0c\n\x08MOBILEYE\x10\x12\x12\x0e\n\nDELPHI_ESR\x10\x13\x12\x10\n\x0cSTORYTELLING\x10\x14\x12\x10\n\x0cTASK_MANAGER\x10\x15\"4\n\x08LogLevel\x12\x08\n\x04INFO\x10\x00\x12\x08\n\x04WARN\x10\x01\x12\t\n\x05\x45RROR\x10\x02\x12\t\n\x05\x46\x41TAL\x10\x03\"p\n\x0eMonitorMessage\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12\x37\n\x04item\x18\x02 \x03(\x0b\x32).apollo.common.monitor.MonitorMessageItem'
  ,
  dependencies=[modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2.DESCRIPTOR,])



_MONITORMESSAGEITEM_MESSAGESOURCE = _descriptor.EnumDescriptor(
  name='MessageSource',
  full_name='apollo.common.monitor.MonitorMessageItem.MessageSource',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CANBUS', index=1, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CONTROL', index=2, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DECISION', index=3, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='LOCALIZATION', index=4, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PLANNING', index=5, number=6,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PREDICTION', index=6, number=7,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SIMULATOR', index=7, number=8,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='HWSYS', index=8, number=9,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ROUTING', index=9, number=10,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MONITOR', index=10, number=11,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='HMI', index=11, number=12,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='RELATIVE_MAP', index=12, number=13,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='GNSS', index=13, number=14,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CONTI_RADAR', index=14, number=15,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='RACOBIT_RADAR', index=15, number=16,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ULTRASONIC_RADAR', index=16, number=17,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MOBILEYE', index=17, number=18,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DELPHI_ESR', index=18, number=19,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='STORYTELLING', index=19, number=20,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='TASK_MANAGER', index=20, number=21,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=318,
  serialized_end=646,
)
_sym_db.RegisterEnumDescriptor(_MONITORMESSAGEITEM_MESSAGESOURCE)

_MONITORMESSAGEITEM_LOGLEVEL = _descriptor.EnumDescriptor(
  name='LogLevel',
  full_name='apollo.common.monitor.MonitorMessageItem.LogLevel',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='INFO', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='WARN', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ERROR', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='FATAL', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=648,
  serialized_end=700,
)
_sym_db.RegisterEnumDescriptor(_MONITORMESSAGEITEM_LOGLEVEL)


_MONITORMESSAGEITEM = _descriptor.Descriptor(
  name='MonitorMessageItem',
  full_name='apollo.common.monitor.MonitorMessageItem',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='source', full_name='apollo.common.monitor.MonitorMessageItem.source', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='msg', full_name='apollo.common.monitor.MonitorMessageItem.msg', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='log_level', full_name='apollo.common.monitor.MonitorMessageItem.log_level', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _MONITORMESSAGEITEM_MESSAGESOURCE,
    _MONITORMESSAGEITEM_LOGLEVEL,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=123,
  serialized_end=700,
)


_MONITORMESSAGE = _descriptor.Descriptor(
  name='MonitorMessage',
  full_name='apollo.common.monitor.MonitorMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.common.monitor.MonitorMessage.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='item', full_name='apollo.common.monitor.MonitorMessage.item', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=702,
  serialized_end=814,
)

_MONITORMESSAGEITEM.fields_by_name['source'].enum_type = _MONITORMESSAGEITEM_MESSAGESOURCE
_MONITORMESSAGEITEM.fields_by_name['log_level'].enum_type = _MONITORMESSAGEITEM_LOGLEVEL
_MONITORMESSAGEITEM_MESSAGESOURCE.containing_type = _MONITORMESSAGEITEM
_MONITORMESSAGEITEM_LOGLEVEL.containing_type = _MONITORMESSAGEITEM
_MONITORMESSAGE.fields_by_name['header'].message_type = modules_dot_common__msgs_dot_basic__msgs_dot_header__pb2._HEADER
_MONITORMESSAGE.fields_by_name['item'].message_type = _MONITORMESSAGEITEM
DESCRIPTOR.message_types_by_name['MonitorMessageItem'] = _MONITORMESSAGEITEM
DESCRIPTOR.message_types_by_name['MonitorMessage'] = _MONITORMESSAGE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MonitorMessageItem = _reflection.GeneratedProtocolMessageType('MonitorMessageItem', (_message.Message,), {
  'DESCRIPTOR' : _MONITORMESSAGEITEM,
  '__module__' : 'modules.common_msgs.monitor_msgs.monitor_log_pb2'
  # @@protoc_insertion_point(class_scope:apollo.common.monitor.MonitorMessageItem)
  })
_sym_db.RegisterMessage(MonitorMessageItem)

MonitorMessage = _reflection.GeneratedProtocolMessageType('MonitorMessage', (_message.Message,), {
  'DESCRIPTOR' : _MONITORMESSAGE,
  '__module__' : 'modules.common_msgs.monitor_msgs.monitor_log_pb2'
  # @@protoc_insertion_point(class_scope:apollo.common.monitor.MonitorMessage)
  })
_sym_db.RegisterMessage(MonitorMessage)


# @@protoc_insertion_point(module_scope)
