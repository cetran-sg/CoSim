# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/pipeline/proto/stage/mask_pillars_detection_config.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/perception/pipeline/proto/stage/mask_pillars_detection_config.proto',
  package='apollo.perception.lidar',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\nKmodules/perception/pipeline/proto/stage/mask_pillars_detection_config.proto\x12\x17\x61pollo.perception.lidar\"\x1c\n\x1aMaskPillarsDetectionConfig'
)




_MASKPILLARSDETECTIONCONFIG = _descriptor.Descriptor(
  name='MaskPillarsDetectionConfig',
  full_name='apollo.perception.lidar.MaskPillarsDetectionConfig',
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
  serialized_start=104,
  serialized_end=132,
)

DESCRIPTOR.message_types_by_name['MaskPillarsDetectionConfig'] = _MASKPILLARSDETECTIONCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MaskPillarsDetectionConfig = _reflection.GeneratedProtocolMessageType('MaskPillarsDetectionConfig', (_message.Message,), {
  'DESCRIPTOR' : _MASKPILLARSDETECTIONCONFIG,
  '__module__' : 'modules.perception.pipeline.proto.stage.mask_pillars_detection_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.lidar.MaskPillarsDetectionConfig)
  })
_sym_db.RegisterMessage(MaskPillarsDetectionConfig)


# @@protoc_insertion_point(module_scope)