# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/drivers/radar/conti_radar/proto/conti_radar_conf.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common_msgs.sensor_msgs import conti_radar_pb2 as modules_dot_common__msgs_dot_sensor__msgs_dot_conti__radar__pb2
from modules.common_msgs.drivers_msgs import can_card_parameter_pb2 as modules_dot_common__msgs_dot_drivers__msgs_dot_can__card__parameter__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/drivers/radar/conti_radar/proto/conti_radar_conf.proto',
  package='apollo.drivers.conti_radar',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n>modules/drivers/radar/conti_radar/proto/conti_radar_conf.proto\x12\x1a\x61pollo.drivers.conti_radar\x1a\x31modules/common_msgs/sensor_msgs/conti_radar.proto\x1a\x39modules/common_msgs/drivers_msgs/can_card_parameter.proto\"\xb6\x01\n\x07\x43\x61nConf\x12\x43\n\x12\x63\x61n_card_parameter\x18\x01 \x01(\x0b\x32\'.apollo.drivers.canbus.CANCardParameter\x12 \n\x11\x65nable_debug_mode\x18\x02 \x01(\x08:\x05\x66\x61lse\x12\"\n\x13\x65nable_receiver_log\x18\x03 \x01(\x08:\x05\x66\x61lse\x12 \n\x11\x65nable_sender_log\x18\x04 \x01(\x08:\x05\x66\x61lse\"\xf4\x05\n\tRadarConf\x12!\n\x12max_distance_valid\x18\x01 \x01(\x08:\x05\x66\x61lse\x12\x1e\n\x0fsensor_id_valid\x18\x02 \x01(\x08:\x05\x66\x61lse\x12 \n\x11radar_power_valid\x18\x03 \x01(\x08:\x05\x66\x61lse\x12\x1f\n\x11output_type_valid\x18\x04 \x01(\x08:\x04true\x12 \n\x12send_quality_valid\x18\x05 \x01(\x08:\x04true\x12!\n\x13send_ext_info_valid\x18\x06 \x01(\x08:\x04true\x12\x1f\n\x10sort_index_valid\x18\x07 \x01(\x08:\x05\x66\x61lse\x12 \n\x12store_in_nvm_valid\x18\x08 \x01(\x08:\x04true\x12\x1f\n\x10\x63trl_relay_valid\x18\t \x01(\x08:\x05\x66\x61lse\x12!\n\x13rcs_threshold_valid\x18\n \x01(\x08:\x04true\x12\x19\n\x0cmax_distance\x18\x0b \x01(\r:\x03\x32\x34\x38\x12\x14\n\tsensor_id\x18\x0c \x01(\r:\x01\x30\x12S\n\x0boutput_type\x18\r \x01(\x0e\x32).apollo.drivers.RadarState_201.OutputType:\x13OUTPUT_TYPE_OBJECTS\x12\x16\n\x0bradar_power\x18\x0e \x01(\r:\x01\x30\x12\x15\n\nctrl_relay\x18\x0f \x01(\r:\x01\x30\x12\x1b\n\rsend_ext_info\x18\x10 \x01(\x08:\x04true\x12\x1a\n\x0csend_quality\x18\x11 \x01(\x08:\x04true\x12\x15\n\nsort_index\x18\x12 \x01(\r:\x01\x30\x12\x17\n\x0cstore_in_nvm\x18\x13 \x01(\r:\x01\x31\x12Z\n\rrcs_threshold\x18\x14 \x01(\x0e\x32+.apollo.drivers.RadarState_201.RcsThreshold:\x16RCS_THRESHOLD_STANDARD\x12\x1b\n\x13input_send_interval\x18\x15 \x01(\x04\"\x99\x01\n\x0e\x43ontiRadarConf\x12\x35\n\x08\x63\x61n_conf\x18\x01 \x01(\x0b\x32#.apollo.drivers.conti_radar.CanConf\x12\x39\n\nradar_conf\x18\x02 \x01(\x0b\x32%.apollo.drivers.conti_radar.RadarConf\x12\x15\n\rradar_channel\x18\x03 \x01(\t'
  ,
  dependencies=[modules_dot_common__msgs_dot_sensor__msgs_dot_conti__radar__pb2.DESCRIPTOR,modules_dot_common__msgs_dot_drivers__msgs_dot_can__card__parameter__pb2.DESCRIPTOR,])




_CANCONF = _descriptor.Descriptor(
  name='CanConf',
  full_name='apollo.drivers.conti_radar.CanConf',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='can_card_parameter', full_name='apollo.drivers.conti_radar.CanConf.can_card_parameter', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='enable_debug_mode', full_name='apollo.drivers.conti_radar.CanConf.enable_debug_mode', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='enable_receiver_log', full_name='apollo.drivers.conti_radar.CanConf.enable_receiver_log', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='enable_sender_log', full_name='apollo.drivers.conti_radar.CanConf.enable_sender_log', index=3,
      number=4, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
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
  serialized_start=205,
  serialized_end=387,
)


_RADARCONF = _descriptor.Descriptor(
  name='RadarConf',
  full_name='apollo.drivers.conti_radar.RadarConf',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='max_distance_valid', full_name='apollo.drivers.conti_radar.RadarConf.max_distance_valid', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='sensor_id_valid', full_name='apollo.drivers.conti_radar.RadarConf.sensor_id_valid', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='radar_power_valid', full_name='apollo.drivers.conti_radar.RadarConf.radar_power_valid', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='output_type_valid', full_name='apollo.drivers.conti_radar.RadarConf.output_type_valid', index=3,
      number=4, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='send_quality_valid', full_name='apollo.drivers.conti_radar.RadarConf.send_quality_valid', index=4,
      number=5, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='send_ext_info_valid', full_name='apollo.drivers.conti_radar.RadarConf.send_ext_info_valid', index=5,
      number=6, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='sort_index_valid', full_name='apollo.drivers.conti_radar.RadarConf.sort_index_valid', index=6,
      number=7, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='store_in_nvm_valid', full_name='apollo.drivers.conti_radar.RadarConf.store_in_nvm_valid', index=7,
      number=8, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='ctrl_relay_valid', full_name='apollo.drivers.conti_radar.RadarConf.ctrl_relay_valid', index=8,
      number=9, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='rcs_threshold_valid', full_name='apollo.drivers.conti_radar.RadarConf.rcs_threshold_valid', index=9,
      number=10, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_distance', full_name='apollo.drivers.conti_radar.RadarConf.max_distance', index=10,
      number=11, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=248,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='sensor_id', full_name='apollo.drivers.conti_radar.RadarConf.sensor_id', index=11,
      number=12, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='output_type', full_name='apollo.drivers.conti_radar.RadarConf.output_type', index=12,
      number=13, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='radar_power', full_name='apollo.drivers.conti_radar.RadarConf.radar_power', index=13,
      number=14, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='ctrl_relay', full_name='apollo.drivers.conti_radar.RadarConf.ctrl_relay', index=14,
      number=15, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='send_ext_info', full_name='apollo.drivers.conti_radar.RadarConf.send_ext_info', index=15,
      number=16, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='send_quality', full_name='apollo.drivers.conti_radar.RadarConf.send_quality', index=16,
      number=17, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='sort_index', full_name='apollo.drivers.conti_radar.RadarConf.sort_index', index=17,
      number=18, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='store_in_nvm', full_name='apollo.drivers.conti_radar.RadarConf.store_in_nvm', index=18,
      number=19, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='rcs_threshold', full_name='apollo.drivers.conti_radar.RadarConf.rcs_threshold', index=19,
      number=20, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='input_send_interval', full_name='apollo.drivers.conti_radar.RadarConf.input_send_interval', index=20,
      number=21, type=4, cpp_type=4, label=1,
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
  serialized_start=390,
  serialized_end=1146,
)


_CONTIRADARCONF = _descriptor.Descriptor(
  name='ContiRadarConf',
  full_name='apollo.drivers.conti_radar.ContiRadarConf',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='can_conf', full_name='apollo.drivers.conti_radar.ContiRadarConf.can_conf', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='radar_conf', full_name='apollo.drivers.conti_radar.ContiRadarConf.radar_conf', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='radar_channel', full_name='apollo.drivers.conti_radar.ContiRadarConf.radar_channel', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
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
  serialized_start=1149,
  serialized_end=1302,
)

_CANCONF.fields_by_name['can_card_parameter'].message_type = modules_dot_common__msgs_dot_drivers__msgs_dot_can__card__parameter__pb2._CANCARDPARAMETER
_RADARCONF.fields_by_name['output_type'].enum_type = modules_dot_common__msgs_dot_sensor__msgs_dot_conti__radar__pb2._RADARSTATE_201_OUTPUTTYPE
_RADARCONF.fields_by_name['rcs_threshold'].enum_type = modules_dot_common__msgs_dot_sensor__msgs_dot_conti__radar__pb2._RADARSTATE_201_RCSTHRESHOLD
_CONTIRADARCONF.fields_by_name['can_conf'].message_type = _CANCONF
_CONTIRADARCONF.fields_by_name['radar_conf'].message_type = _RADARCONF
DESCRIPTOR.message_types_by_name['CanConf'] = _CANCONF
DESCRIPTOR.message_types_by_name['RadarConf'] = _RADARCONF
DESCRIPTOR.message_types_by_name['ContiRadarConf'] = _CONTIRADARCONF
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

CanConf = _reflection.GeneratedProtocolMessageType('CanConf', (_message.Message,), {
  'DESCRIPTOR' : _CANCONF,
  '__module__' : 'modules.drivers.radar.conti_radar.proto.conti_radar_conf_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.conti_radar.CanConf)
  })
_sym_db.RegisterMessage(CanConf)

RadarConf = _reflection.GeneratedProtocolMessageType('RadarConf', (_message.Message,), {
  'DESCRIPTOR' : _RADARCONF,
  '__module__' : 'modules.drivers.radar.conti_radar.proto.conti_radar_conf_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.conti_radar.RadarConf)
  })
_sym_db.RegisterMessage(RadarConf)

ContiRadarConf = _reflection.GeneratedProtocolMessageType('ContiRadarConf', (_message.Message,), {
  'DESCRIPTOR' : _CONTIRADARCONF,
  '__module__' : 'modules.drivers.radar.conti_radar.proto.conti_radar_conf_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.conti_radar.ContiRadarConf)
  })
_sym_db.RegisterMessage(ContiRadarConf)


# @@protoc_insertion_point(module_scope)
