# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/pipeline/proto/plugin/recover_bbox_config.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/perception/pipeline/proto/plugin/recover_bbox_config.proto',
  package='apollo.perception.camera',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\nBmodules/perception/pipeline/proto/plugin/recover_bbox_config.proto\x12\x18\x61pollo.perception.camera\"C\n\x11RecoverBboxConfig\x12\r\n\x05roi_w\x18\x01 \x01(\x05\x12\r\n\x05roi_h\x18\x02 \x01(\x05\x12\x10\n\x08offset_y\x18\x03 \x01(\x05'
)




_RECOVERBBOXCONFIG = _descriptor.Descriptor(
  name='RecoverBboxConfig',
  full_name='apollo.perception.camera.RecoverBboxConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='roi_w', full_name='apollo.perception.camera.RecoverBboxConfig.roi_w', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='roi_h', full_name='apollo.perception.camera.RecoverBboxConfig.roi_h', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='offset_y', full_name='apollo.perception.camera.RecoverBboxConfig.offset_y', index=2,
      number=3, type=5, cpp_type=1, label=1,
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
  serialized_start=96,
  serialized_end=163,
)

DESCRIPTOR.message_types_by_name['RecoverBboxConfig'] = _RECOVERBBOXCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

RecoverBboxConfig = _reflection.GeneratedProtocolMessageType('RecoverBboxConfig', (_message.Message,), {
  'DESCRIPTOR' : _RECOVERBBOXCONFIG,
  '__module__' : 'modules.perception.pipeline.proto.plugin.recover_bbox_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.camera.RecoverBboxConfig)
  })
_sym_db.RegisterMessage(RecoverBboxConfig)


# @@protoc_insertion_point(module_scope)
