# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/drivers/video/proto/video_h265cfg.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/drivers/video/proto/video_h265cfg.proto',
  package='apollo.drivers.video.config',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n/modules/drivers/video/proto/video_h265cfg.proto\x12\x1b\x61pollo.drivers.video.config\"\xf0\x05\n\x10\x43\x61meraH265Config\x12\x10\n\x08udp_port\x18\x01 \x02(\r\x12\x10\n\x08\x66rame_id\x18\x02 \x02(\t\x12\x1a\n\x0cpixel_format\x18\x03 \x02(\t:\x04yuyv\x12\x0e\n\x06record\x18\x04 \x02(\r\x12\r\n\x05width\x18\x05 \x02(\r\x12\x0e\n\x06height\x18\x06 \x02(\r\x12\x12\n\nframe_rate\x18\x07 \x02(\r\x12\x19\n\nmonochrome\x18\x08 \x02(\x08:\x05\x66\x61lse\x12\x16\n\nbrightness\x18\t \x02(\x05:\x02-1\x12\x14\n\x08\x63ontrast\x18\n \x02(\x05:\x02-1\x12\x16\n\nsaturation\x18\x0b \x02(\x05:\x02-1\x12\x15\n\tsharpness\x18\x0c \x02(\x05:\x02-1\x12\x10\n\x04gain\x18\r \x02(\x05:\x02-1\x12\x19\n\nauto_focus\x18\x0e \x02(\x08:\x05\x66\x61lse\x12\x11\n\x05\x66ocus\x18\x0f \x02(\x05:\x02-1\x12\x1b\n\rauto_exposure\x18\x10 \x02(\x08:\x04true\x12\x15\n\x08\x65xposure\x18\x11 \x02(\x05:\x03\x31\x30\x30\x12 \n\x12\x61uto_white_balance\x18\x12 \x02(\x08:\x04true\x12\x1b\n\rwhite_balance\x18\x13 \x02(\x05:\x04\x34\x30\x30\x30\x12\x1a\n\x0f\x62ytes_per_pixel\x18\x14 \x02(\r:\x01\x33\x12\x1b\n\rtrigger_param\x18\x15 \x02(\t:\x04\x66\x32\x66\x66\x12\x1d\n\x11metric_error_code\x18\x16 \x02(\r:\x02\x31\x31\x12\x1b\n\x0f\x66pga_dev_number\x18\x17 \x02(\x05:\x02-1\x12\x1d\n\x11\x63\x61mera_seq_number\x18\x18 \x02(\x05:\x02-1\x12S\n\rcompress_conf\x18\x19 \x01(\x0b\x32<.apollo.drivers.video.config.CameraH265Config.CompressConfig\x1a\x45\n\x0e\x43ompressConfig\x12\x16\n\x0eoutput_channel\x18\x01 \x01(\t\x12\x1b\n\x0fimage_pool_size\x18\x02 \x01(\r:\x02\x32\x30'
)




_CAMERAH265CONFIG_COMPRESSCONFIG = _descriptor.Descriptor(
  name='CompressConfig',
  full_name='apollo.drivers.video.config.CameraH265Config.CompressConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='output_channel', full_name='apollo.drivers.video.config.CameraH265Config.CompressConfig.output_channel', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='image_pool_size', full_name='apollo.drivers.video.config.CameraH265Config.CompressConfig.image_pool_size', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=20,
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
  serialized_start=764,
  serialized_end=833,
)

_CAMERAH265CONFIG = _descriptor.Descriptor(
  name='CameraH265Config',
  full_name='apollo.drivers.video.config.CameraH265Config',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='udp_port', full_name='apollo.drivers.video.config.CameraH265Config.udp_port', index=0,
      number=1, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='frame_id', full_name='apollo.drivers.video.config.CameraH265Config.frame_id', index=1,
      number=2, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pixel_format', full_name='apollo.drivers.video.config.CameraH265Config.pixel_format', index=2,
      number=3, type=9, cpp_type=9, label=2,
      has_default_value=True, default_value=b"yuyv".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='record', full_name='apollo.drivers.video.config.CameraH265Config.record', index=3,
      number=4, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='width', full_name='apollo.drivers.video.config.CameraH265Config.width', index=4,
      number=5, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='height', full_name='apollo.drivers.video.config.CameraH265Config.height', index=5,
      number=6, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='frame_rate', full_name='apollo.drivers.video.config.CameraH265Config.frame_rate', index=6,
      number=7, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='monochrome', full_name='apollo.drivers.video.config.CameraH265Config.monochrome', index=7,
      number=8, type=8, cpp_type=7, label=2,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='brightness', full_name='apollo.drivers.video.config.CameraH265Config.brightness', index=8,
      number=9, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='contrast', full_name='apollo.drivers.video.config.CameraH265Config.contrast', index=9,
      number=10, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='saturation', full_name='apollo.drivers.video.config.CameraH265Config.saturation', index=10,
      number=11, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='sharpness', full_name='apollo.drivers.video.config.CameraH265Config.sharpness', index=11,
      number=12, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='gain', full_name='apollo.drivers.video.config.CameraH265Config.gain', index=12,
      number=13, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='auto_focus', full_name='apollo.drivers.video.config.CameraH265Config.auto_focus', index=13,
      number=14, type=8, cpp_type=7, label=2,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='focus', full_name='apollo.drivers.video.config.CameraH265Config.focus', index=14,
      number=15, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='auto_exposure', full_name='apollo.drivers.video.config.CameraH265Config.auto_exposure', index=15,
      number=16, type=8, cpp_type=7, label=2,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='exposure', full_name='apollo.drivers.video.config.CameraH265Config.exposure', index=16,
      number=17, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=100,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='auto_white_balance', full_name='apollo.drivers.video.config.CameraH265Config.auto_white_balance', index=17,
      number=18, type=8, cpp_type=7, label=2,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='white_balance', full_name='apollo.drivers.video.config.CameraH265Config.white_balance', index=18,
      number=19, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=4000,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='bytes_per_pixel', full_name='apollo.drivers.video.config.CameraH265Config.bytes_per_pixel', index=19,
      number=20, type=13, cpp_type=3, label=2,
      has_default_value=True, default_value=3,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='trigger_param', full_name='apollo.drivers.video.config.CameraH265Config.trigger_param', index=20,
      number=21, type=9, cpp_type=9, label=2,
      has_default_value=True, default_value=b"f2ff".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='metric_error_code', full_name='apollo.drivers.video.config.CameraH265Config.metric_error_code', index=21,
      number=22, type=13, cpp_type=3, label=2,
      has_default_value=True, default_value=11,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='fpga_dev_number', full_name='apollo.drivers.video.config.CameraH265Config.fpga_dev_number', index=22,
      number=23, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='camera_seq_number', full_name='apollo.drivers.video.config.CameraH265Config.camera_seq_number', index=23,
      number=24, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='compress_conf', full_name='apollo.drivers.video.config.CameraH265Config.compress_conf', index=24,
      number=25, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_CAMERAH265CONFIG_COMPRESSCONFIG, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=81,
  serialized_end=833,
)

_CAMERAH265CONFIG_COMPRESSCONFIG.containing_type = _CAMERAH265CONFIG
_CAMERAH265CONFIG.fields_by_name['compress_conf'].message_type = _CAMERAH265CONFIG_COMPRESSCONFIG
DESCRIPTOR.message_types_by_name['CameraH265Config'] = _CAMERAH265CONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

CameraH265Config = _reflection.GeneratedProtocolMessageType('CameraH265Config', (_message.Message,), {

  'CompressConfig' : _reflection.GeneratedProtocolMessageType('CompressConfig', (_message.Message,), {
    'DESCRIPTOR' : _CAMERAH265CONFIG_COMPRESSCONFIG,
    '__module__' : 'modules.drivers.video.proto.video_h265cfg_pb2'
    # @@protoc_insertion_point(class_scope:apollo.drivers.video.config.CameraH265Config.CompressConfig)
    })
  ,
  'DESCRIPTOR' : _CAMERAH265CONFIG,
  '__module__' : 'modules.drivers.video.proto.video_h265cfg_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.video.config.CameraH265Config)
  })
_sym_db.RegisterMessage(CameraH265Config)
_sym_db.RegisterMessage(CameraH265Config.CompressConfig)


# @@protoc_insertion_point(module_scope)
