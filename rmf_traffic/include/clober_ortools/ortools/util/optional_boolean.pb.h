// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ortools/util/optional_boolean.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ortools_2futil_2foptional_5fboolean_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ortools_2futil_2foptional_5fboolean_2eproto

#include <limits>
#include <string>

#include <clober_ortools/google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3015000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3015008 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <clober_ortools/google/protobuf/port_undef.inc>
#include <clober_ortools/google/protobuf/io/coded_stream.h>
#include <clober_ortools/google/protobuf/arena.h>
#include <clober_ortools/google/protobuf/arenastring.h>
#include <clober_ortools/google/protobuf/generated_message_table_driven.h>
#include <clober_ortools/google/protobuf/generated_message_util.h>
#include <clober_ortools/google/protobuf/metadata_lite.h>
#include <clober_ortools/google/protobuf/generated_message_reflection.h>
#include <clober_ortools/google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <clober_ortools/google/protobuf/extension_set.h>  // IWYU pragma: export
#include <clober_ortools/google/protobuf/generated_enum_reflection.h>
// @@protoc_insertion_point(includes)
#include <clober_ortools/google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ortools_2futil_2foptional_5fboolean_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ortools_2futil_2foptional_5fboolean_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ortools_2futil_2foptional_5fboolean_2eproto;
::PROTOBUF_NAMESPACE_ID::Metadata descriptor_table_ortools_2futil_2foptional_5fboolean_2eproto_metadata_getter(int index);
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE
namespace operations_research {

enum OptionalBoolean : int {
  BOOL_UNSPECIFIED = 0,
  BOOL_FALSE = 2,
  BOOL_TRUE = 3,
  OptionalBoolean_INT_MIN_SENTINEL_DO_NOT_USE_ = std::numeric_limits<::PROTOBUF_NAMESPACE_ID::int32>::min(),
  OptionalBoolean_INT_MAX_SENTINEL_DO_NOT_USE_ = std::numeric_limits<::PROTOBUF_NAMESPACE_ID::int32>::max()
};
bool OptionalBoolean_IsValid(int value);
constexpr OptionalBoolean OptionalBoolean_MIN = BOOL_UNSPECIFIED;
constexpr OptionalBoolean OptionalBoolean_MAX = BOOL_TRUE;
constexpr int OptionalBoolean_ARRAYSIZE = OptionalBoolean_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* OptionalBoolean_descriptor();
template<typename T>
inline const std::string& OptionalBoolean_Name(T enum_t_value) {
  static_assert(::std::is_same<T, OptionalBoolean>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function OptionalBoolean_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    OptionalBoolean_descriptor(), enum_t_value);
}
inline bool OptionalBoolean_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, OptionalBoolean* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<OptionalBoolean>(
    OptionalBoolean_descriptor(), name, value);
}
// ===================================================================


// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace operations_research

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::operations_research::OptionalBoolean> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::operations_research::OptionalBoolean>() {
  return ::operations_research::OptionalBoolean_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <clober_ortools/google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ortools_2futil_2foptional_5fboolean_2eproto
