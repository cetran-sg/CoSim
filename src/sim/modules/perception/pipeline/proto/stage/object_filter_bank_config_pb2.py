# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/pipeline/proto/stage/object_filter_bank_config.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/perception/pipeline/proto/stage/object_filter_bank_config.proto',
  package='apollo.perception.lidar',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\nGmodules/perception/pipeline/proto/stage/object_filter_bank_config.proto\x12\x17\x61pollo.perception.lidar\"-\n\x16ObjectFilterBankConfig\x12\x13\n\x0b\x66ilter_name\x18\x01 \x03(\t'
)




_OBJECTFILTERBANKCONFIG = _descriptor.Descriptor(
  name='ObjectFilterBankConfig',
  full_name='apollo.perception.lidar.ObjectFilterBankConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='filter_name', full_name='apollo.perception.lidar.ObjectFilterBankConfig.filter_name', index=0,
      number=1, type=9, cpp_type=9, label=3,
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
  serialized_start=100,
  serialized_end=145,
)

DESCRIPTOR.message_types_by_name['ObjectFilterBankConfig'] = _OBJECTFILTERBANKCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ObjectFilterBankConfig = _reflection.GeneratedProtocolMessageType('ObjectFilterBankConfig', (_message.Message,), {
  'DESCRIPTOR' : _OBJECTFILTERBANKCONFIG,
  '__module__' : 'modules.perception.pipeline.proto.stage.object_filter_bank_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.lidar.ObjectFilterBankConfig)
  })
_sym_db.RegisterMessage(ObjectFilterBankConfig)


# @@protoc_insertion_point(module_scope)