# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/lidar/lib/detector/cnn_segmentation/proto/spp_engine_config.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/perception/lidar/lib/detector/cnn_segmentation/proto/spp_engine_config.proto',
  package='apollo.perception.lidar',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\nTmodules/perception/lidar/lib/detector/cnn_segmentation/proto/spp_engine_config.proto\x12\x17\x61pollo.perception.lidar\"*\n\x0fSppEngineConfig\x12\x17\n\nheight_gap\x18\x08 \x01(\x02:\x03\x30.5'
)




_SPPENGINECONFIG = _descriptor.Descriptor(
  name='SppEngineConfig',
  full_name='apollo.perception.lidar.SppEngineConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='height_gap', full_name='apollo.perception.lidar.SppEngineConfig.height_gap', index=0,
      number=8, type=2, cpp_type=6, label=1,
      has_default_value=True, default_value=float(0.5),
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
  serialized_end=155,
)

DESCRIPTOR.message_types_by_name['SppEngineConfig'] = _SPPENGINECONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SppEngineConfig = _reflection.GeneratedProtocolMessageType('SppEngineConfig', (_message.Message,), {
  'DESCRIPTOR' : _SPPENGINECONFIG,
  '__module__' : 'modules.perception.lidar.lib.detector.cnn_segmentation.proto.spp_engine_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.lidar.SppEngineConfig)
  })
_sym_db.RegisterMessage(SppEngineConfig)


# @@protoc_insertion_point(module_scope)
