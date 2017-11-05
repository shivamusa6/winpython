# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ortools/constraint_solver/solver_parameters.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ortools/constraint_solver/solver_parameters.proto',
  package='operations_research',
  syntax='proto3',
  serialized_pb=_b('\n1ortools/constraint_solver/solver_parameters.proto\x12\x13operations_research\"\xee\x07\n\x1a\x43onstraintSolverParameters\x12X\n\x0e\x63ompress_trail\x18\x01 \x01(\x0e\x32@.operations_research.ConstraintSolverParameters.TrailCompression\x12\x18\n\x10trail_block_size\x18\x02 \x01(\x05\x12\x18\n\x10\x61rray_split_size\x18\x03 \x01(\x05\x12\x13\n\x0bstore_names\x18\x04 \x01(\x08\x12\x1b\n\x13name_cast_variables\x18\x05 \x01(\x08\x12\x1a\n\x12name_all_variables\x18\x06 \x01(\x08\x12\x1b\n\x13profile_propagation\x18\x07 \x01(\x08\x12\x14\n\x0cprofile_file\x18\x08 \x01(\t\x12\x1c\n\x14profile_local_search\x18\x10 \x01(\x08\x12\"\n\x1aprint_local_search_profile\x18\x11 \x01(\x08\x12\x19\n\x11trace_propagation\x18\t \x01(\x08\x12\x14\n\x0ctrace_search\x18\n \x01(\x08\x12\x13\n\x0bprint_model\x18\x0b \x01(\x08\x12\x19\n\x11print_model_stats\x18\x0c \x01(\x08\x12\x1f\n\x17print_added_constraints\x18\r \x01(\x08\x12\x13\n\x0b\x65xport_file\x18\x0e \x01(\t\x12\x15\n\rdisable_solve\x18\x0f \x01(\x08\x12\x19\n\x11use_compact_table\x18\x64 \x01(\x08\x12\x17\n\x0fuse_small_table\x18\x65 \x01(\x08\x12\x15\n\ruse_sat_table\x18\x66 \x01(\x08\x12\x1c\n\x14\x61\x63\x34r_table_threshold\x18g \x01(\x05\x12\x15\n\ruse_mdd_table\x18h \x01(\x08\x12\"\n\x1ause_cumulative_edge_finder\x18i \x01(\x08\x12!\n\x19use_cumulative_time_table\x18j \x01(\x08\x12&\n\x1euse_cumulative_time_table_sync\x18p \x01(\x08\x12&\n\x1euse_sequence_high_demand_tasks\x18k \x01(\x08\x12%\n\x1duse_all_possible_disjunctions\x18l \x01(\x08\x12\x1c\n\x14max_edge_finder_size\x18m \x01(\x05\x12\x1c\n\x14\x64iffn_use_cumulative\x18n \x01(\x08\x12\x17\n\x0fuse_element_rmq\x18o \x01(\x08\">\n\x10TrailCompression\x12\x12\n\x0eNO_COMPRESSION\x10\x00\x12\x16\n\x12\x43OMPRESS_WITH_ZLIB\x10\x01\x42I\n#com.google.ortools.constraintsolverP\x01\xaa\x02\x1fGoogle.OrTools.ConstraintSolverb\x06proto3')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_CONSTRAINTSOLVERPARAMETERS_TRAILCOMPRESSION = _descriptor.EnumDescriptor(
  name='TrailCompression',
  full_name='operations_research.ConstraintSolverParameters.TrailCompression',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NO_COMPRESSION', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COMPRESS_WITH_ZLIB', index=1, number=1,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=1019,
  serialized_end=1081,
)
_sym_db.RegisterEnumDescriptor(_CONSTRAINTSOLVERPARAMETERS_TRAILCOMPRESSION)


_CONSTRAINTSOLVERPARAMETERS = _descriptor.Descriptor(
  name='ConstraintSolverParameters',
  full_name='operations_research.ConstraintSolverParameters',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='compress_trail', full_name='operations_research.ConstraintSolverParameters.compress_trail', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='trail_block_size', full_name='operations_research.ConstraintSolverParameters.trail_block_size', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='array_split_size', full_name='operations_research.ConstraintSolverParameters.array_split_size', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='store_names', full_name='operations_research.ConstraintSolverParameters.store_names', index=3,
      number=4, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='name_cast_variables', full_name='operations_research.ConstraintSolverParameters.name_cast_variables', index=4,
      number=5, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='name_all_variables', full_name='operations_research.ConstraintSolverParameters.name_all_variables', index=5,
      number=6, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='profile_propagation', full_name='operations_research.ConstraintSolverParameters.profile_propagation', index=6,
      number=7, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='profile_file', full_name='operations_research.ConstraintSolverParameters.profile_file', index=7,
      number=8, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='profile_local_search', full_name='operations_research.ConstraintSolverParameters.profile_local_search', index=8,
      number=16, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='print_local_search_profile', full_name='operations_research.ConstraintSolverParameters.print_local_search_profile', index=9,
      number=17, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='trace_propagation', full_name='operations_research.ConstraintSolverParameters.trace_propagation', index=10,
      number=9, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='trace_search', full_name='operations_research.ConstraintSolverParameters.trace_search', index=11,
      number=10, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='print_model', full_name='operations_research.ConstraintSolverParameters.print_model', index=12,
      number=11, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='print_model_stats', full_name='operations_research.ConstraintSolverParameters.print_model_stats', index=13,
      number=12, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='print_added_constraints', full_name='operations_research.ConstraintSolverParameters.print_added_constraints', index=14,
      number=13, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='export_file', full_name='operations_research.ConstraintSolverParameters.export_file', index=15,
      number=14, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='disable_solve', full_name='operations_research.ConstraintSolverParameters.disable_solve', index=16,
      number=15, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='use_compact_table', full_name='operations_research.ConstraintSolverParameters.use_compact_table', index=17,
      number=100, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='use_small_table', full_name='operations_research.ConstraintSolverParameters.use_small_table', index=18,
      number=101, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='use_sat_table', full_name='operations_research.ConstraintSolverParameters.use_sat_table', index=19,
      number=102, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ac4r_table_threshold', full_name='operations_research.ConstraintSolverParameters.ac4r_table_threshold', index=20,
      number=103, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='use_mdd_table', full_name='operations_research.ConstraintSolverParameters.use_mdd_table', index=21,
      number=104, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='use_cumulative_edge_finder', full_name='operations_research.ConstraintSolverParameters.use_cumulative_edge_finder', index=22,
      number=105, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='use_cumulative_time_table', full_name='operations_research.ConstraintSolverParameters.use_cumulative_time_table', index=23,
      number=106, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='use_cumulative_time_table_sync', full_name='operations_research.ConstraintSolverParameters.use_cumulative_time_table_sync', index=24,
      number=112, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='use_sequence_high_demand_tasks', full_name='operations_research.ConstraintSolverParameters.use_sequence_high_demand_tasks', index=25,
      number=107, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='use_all_possible_disjunctions', full_name='operations_research.ConstraintSolverParameters.use_all_possible_disjunctions', index=26,
      number=108, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='max_edge_finder_size', full_name='operations_research.ConstraintSolverParameters.max_edge_finder_size', index=27,
      number=109, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='diffn_use_cumulative', full_name='operations_research.ConstraintSolverParameters.diffn_use_cumulative', index=28,
      number=110, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='use_element_rmq', full_name='operations_research.ConstraintSolverParameters.use_element_rmq', index=29,
      number=111, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _CONSTRAINTSOLVERPARAMETERS_TRAILCOMPRESSION,
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=75,
  serialized_end=1081,
)

_CONSTRAINTSOLVERPARAMETERS.fields_by_name['compress_trail'].enum_type = _CONSTRAINTSOLVERPARAMETERS_TRAILCOMPRESSION
_CONSTRAINTSOLVERPARAMETERS_TRAILCOMPRESSION.containing_type = _CONSTRAINTSOLVERPARAMETERS
DESCRIPTOR.message_types_by_name['ConstraintSolverParameters'] = _CONSTRAINTSOLVERPARAMETERS

ConstraintSolverParameters = _reflection.GeneratedProtocolMessageType('ConstraintSolverParameters', (_message.Message,), dict(
  DESCRIPTOR = _CONSTRAINTSOLVERPARAMETERS,
  __module__ = 'ortools.constraint_solver.solver_parameters_pb2'
  # @@protoc_insertion_point(class_scope:operations_research.ConstraintSolverParameters)
  ))
_sym_db.RegisterMessage(ConstraintSolverParameters)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n#com.google.ortools.constraintsolverP\001\252\002\037Google.OrTools.ConstraintSolver'))
# @@protoc_insertion_point(module_scope)
