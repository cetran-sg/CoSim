# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/pipeline/proto/stage/caddn_detection_config.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/perception/pipeline/proto/stage/caddn_detection_config.proto',
  package='apollo.perception.camera',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\nDmodules/perception/pipeline/proto/stage/caddn_detection_config.proto\x12\x18\x61pollo.perception.camera\"\x16\n\x14\x43\x61\x64\x64nDetectionConfig'
)




_CADDNDETECTIONCONFIG = _descriptor.Descriptor(
  name='CaddnDetectionConfig',
  full_name='apollo.perception.camera.CaddnDetectionConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
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
  serialized_start=98,
  serialized_end=120,
)

DESCRIPTOR.message_types_by_name['CaddnDetectionConfig'] = _CADDNDETECTIONCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

CaddnDetectionConfig = _reflection.GeneratedProtocolMessageType('CaddnDetectionConfig', (_message.Message,), {
  'DESCRIPTOR' : _CADDNDETECTIONCONFIG,
  '__module__' : 'modules.perception.pipeline.proto.stage.caddn_detection_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.camera.CaddnDetectionConfig)
  })
_sym_db.RegisterMessage(CaddnDetectionConfig)


# @@protoc_insertion_point(module_scope)
