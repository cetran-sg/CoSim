# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/lidar/lib/scene_manager/proto/scene_manager_config.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/perception/lidar/lib/scene_manager/proto/scene_manager_config.proto',
  package='apollo.perception.lidar',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\nKmodules/perception/lidar/lib/scene_manager/proto/scene_manager_config.proto\x12\x17\x61pollo.perception.lidar\"*\n\x12SceneManagerConfig\x12\x14\n\x0cservice_name\x18\x01 \x03(\t'
)




_SCENEMANAGERCONFIG = _descriptor.Descriptor(
  name='SceneManagerConfig',
  full_name='apollo.perception.lidar.SceneManagerConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='service_name', full_name='apollo.perception.lidar.SceneManagerConfig.service_name', index=0,
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
  serialized_start=104,
  serialized_end=146,
)

DESCRIPTOR.message_types_by_name['SceneManagerConfig'] = _SCENEMANAGERCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SceneManagerConfig = _reflection.GeneratedProtocolMessageType('SceneManagerConfig', (_message.Message,), {
  'DESCRIPTOR' : _SCENEMANAGERCONFIG,
  '__module__' : 'modules.perception.lidar.lib.scene_manager.proto.scene_manager_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.lidar.SceneManagerConfig)
  })
_sym_db.RegisterMessage(SceneManagerConfig)


# @@protoc_insertion_point(module_scope)
