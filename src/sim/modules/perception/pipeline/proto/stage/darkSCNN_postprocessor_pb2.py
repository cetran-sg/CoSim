# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/pipeline/proto/stage/darkSCNN_postprocessor.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/perception/pipeline/proto/stage/darkSCNN_postprocessor.proto',
  package='apollo.perception.camera.lane',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\nDmodules/perception/pipeline/proto/stage/darkSCNN_postprocessor.proto\x12\x1d\x61pollo.perception.camera.lane\"\x92\x02\n\x1e\x44\x61rkSCNNLanePostprocessorParam\x12\x1b\n\x0elane_map_width\x18\x01 \x01(\r:\x03\x36\x34\x30\x12\x1c\n\x0flane_map_height\x18\x02 \x01(\r:\x03\x34\x38\x30\x12\x17\n\nroi_height\x18\x03 \x01(\r:\x03\x37\x36\x38\x12\x16\n\troi_start\x18\x04 \x01(\r:\x03\x33\x31\x32\x12\x17\n\troi_width\x18\x05 \x01(\r:\x04\x31\x39\x32\x30\x12\x19\n\x0einput_offset_x\x18\x06 \x01(\r:\x01\x30\x12\x19\n\x0einput_offset_y\x18\x07 \x01(\r:\x01\x30\x12\x19\n\x0cresize_width\x18\x08 \x01(\r:\x03\x35\x31\x32\x12\x1a\n\rresize_height\x18\t \x01(\r:\x03\x35\x31\x32'
)




_DARKSCNNLANEPOSTPROCESSORPARAM = _descriptor.Descriptor(
  name='DarkSCNNLanePostprocessorParam',
  full_name='apollo.perception.camera.lane.DarkSCNNLanePostprocessorParam',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='lane_map_width', full_name='apollo.perception.camera.lane.DarkSCNNLanePostprocessorParam.lane_map_width', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=640,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='lane_map_height', full_name='apollo.perception.camera.lane.DarkSCNNLanePostprocessorParam.lane_map_height', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=480,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='roi_height', full_name='apollo.perception.camera.lane.DarkSCNNLanePostprocessorParam.roi_height', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=768,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='roi_start', full_name='apollo.perception.camera.lane.DarkSCNNLanePostprocessorParam.roi_start', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=312,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='roi_width', full_name='apollo.perception.camera.lane.DarkSCNNLanePostprocessorParam.roi_width', index=4,
      number=5, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=1920,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='input_offset_x', full_name='apollo.perception.camera.lane.DarkSCNNLanePostprocessorParam.input_offset_x', index=5,
      number=6, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='input_offset_y', full_name='apollo.perception.camera.lane.DarkSCNNLanePostprocessorParam.input_offset_y', index=6,
      number=7, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='resize_width', full_name='apollo.perception.camera.lane.DarkSCNNLanePostprocessorParam.resize_width', index=7,
      number=8, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=512,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='resize_height', full_name='apollo.perception.camera.lane.DarkSCNNLanePostprocessorParam.resize_height', index=8,
      number=9, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=512,
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
  serialized_end=378,
)

DESCRIPTOR.message_types_by_name['DarkSCNNLanePostprocessorParam'] = _DARKSCNNLANEPOSTPROCESSORPARAM
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

DarkSCNNLanePostprocessorParam = _reflection.GeneratedProtocolMessageType('DarkSCNNLanePostprocessorParam', (_message.Message,), {
  'DESCRIPTOR' : _DARKSCNNLANEPOSTPROCESSORPARAM,
  '__module__' : 'modules.perception.pipeline.proto.stage.darkSCNN_postprocessor_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.camera.lane.DarkSCNNLanePostprocessorParam)
  })
_sym_db.RegisterMessage(DarkSCNNLanePostprocessorParam)


# @@protoc_insertion_point(module_scope)
