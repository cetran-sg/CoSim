# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/pipeline/proto/stage/camera_detection_postprocessor_config.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/perception/pipeline/proto/stage/camera_detection_postprocessor_config.proto',
  package='apollo.perception.camera',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\nSmodules/perception/pipeline/proto/stage/camera_detection_postprocessor_config.proto\x12\x18\x61pollo.perception.camera\"$\n\"CameraDetectionPostprocessorConfig'
)




_CAMERADETECTIONPOSTPROCESSORCONFIG = _descriptor.Descriptor(
  name='CameraDetectionPostprocessorConfig',
  full_name='apollo.perception.camera.CameraDetectionPostprocessorConfig',
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
  serialized_start=113,
  serialized_end=149,
)

DESCRIPTOR.message_types_by_name['CameraDetectionPostprocessorConfig'] = _CAMERADETECTIONPOSTPROCESSORCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

CameraDetectionPostprocessorConfig = _reflection.GeneratedProtocolMessageType('CameraDetectionPostprocessorConfig', (_message.Message,), {
  'DESCRIPTOR' : _CAMERADETECTIONPOSTPROCESSORCONFIG,
  '__module__' : 'modules.perception.pipeline.proto.stage.camera_detection_postprocessor_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.camera.CameraDetectionPostprocessorConfig)
  })
_sym_db.RegisterMessage(CameraDetectionPostprocessorConfig)


# @@protoc_insertion_point(module_scope)
