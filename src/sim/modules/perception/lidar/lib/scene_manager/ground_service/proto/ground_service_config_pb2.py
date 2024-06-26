# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/lidar/lib/scene_manager/ground_service/proto/ground_service_config.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/perception/lidar/lib/scene_manager/ground_service/proto/ground_service_config.proto',
  package='apollo.perception.lidar',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n[modules/perception/lidar/lib/scene_manager/ground_service/proto/ground_service_config.proto\x12\x17\x61pollo.perception.lidar\"\\\n\x13GroundServiceConfig\x12\x16\n\troi_rad_x\x18\x01 \x01(\x01:\x03\x31\x32\x30\x12\x16\n\troi_rad_y\x18\x02 \x01(\x01:\x03\x31\x32\x30\x12\x15\n\tgrid_size\x18\x03 \x01(\r:\x02\x31\x36'
)




_GROUNDSERVICECONFIG = _descriptor.Descriptor(
  name='GroundServiceConfig',
  full_name='apollo.perception.lidar.GroundServiceConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='roi_rad_x', full_name='apollo.perception.lidar.GroundServiceConfig.roi_rad_x', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(120),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='roi_rad_y', full_name='apollo.perception.lidar.GroundServiceConfig.roi_rad_y', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(120),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='grid_size', full_name='apollo.perception.lidar.GroundServiceConfig.grid_size', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=16,
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
  serialized_start=120,
  serialized_end=212,
)

DESCRIPTOR.message_types_by_name['GroundServiceConfig'] = _GROUNDSERVICECONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

GroundServiceConfig = _reflection.GeneratedProtocolMessageType('GroundServiceConfig', (_message.Message,), {
  'DESCRIPTOR' : _GROUNDSERVICECONFIG,
  '__module__' : 'modules.perception.lidar.lib.scene_manager.ground_service.proto.ground_service_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.lidar.GroundServiceConfig)
  })
_sym_db.RegisterMessage(GroundServiceConfig)


# @@protoc_insertion_point(module_scope)
