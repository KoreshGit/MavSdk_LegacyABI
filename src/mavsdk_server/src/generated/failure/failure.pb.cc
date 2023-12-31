// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: failure/failure.proto

#include "failure/failure.pb.h"

#include <algorithm>
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/extension_set.h"
#include "google/protobuf/wire_format_lite.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/generated_message_reflection.h"
#include "google/protobuf/reflection_ops.h"
#include "google/protobuf/wire_format.h"
// @@protoc_insertion_point(includes)

// Must be included last.
#include "google/protobuf/port_def.inc"
PROTOBUF_PRAGMA_INIT_SEG
namespace _pb = ::PROTOBUF_NAMESPACE_ID;
namespace _pbi = ::PROTOBUF_NAMESPACE_ID::internal;
namespace mavsdk {
namespace rpc {
namespace failure {
template <typename>
PROTOBUF_CONSTEXPR InjectRequest::InjectRequest(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.failure_unit_)*/ 0

  , /*decltype(_impl_.failure_type_)*/ 0

  , /*decltype(_impl_.instance_)*/ 0

  , /*decltype(_impl_._cached_size_)*/{}} {}
struct InjectRequestDefaultTypeInternal {
  PROTOBUF_CONSTEXPR InjectRequestDefaultTypeInternal() : _instance(::_pbi::ConstantInitialized{}) {}
  ~InjectRequestDefaultTypeInternal() {}
  union {
    InjectRequest _instance;
  };
};

PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT
    PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 InjectRequestDefaultTypeInternal _InjectRequest_default_instance_;
template <typename>
PROTOBUF_CONSTEXPR InjectResponse::InjectResponse(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_._has_bits_)*/{}
  , /*decltype(_impl_._cached_size_)*/{}
  , /*decltype(_impl_.failure_result_)*/nullptr} {}
struct InjectResponseDefaultTypeInternal {
  PROTOBUF_CONSTEXPR InjectResponseDefaultTypeInternal() : _instance(::_pbi::ConstantInitialized{}) {}
  ~InjectResponseDefaultTypeInternal() {}
  union {
    InjectResponse _instance;
  };
};

PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT
    PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 InjectResponseDefaultTypeInternal _InjectResponse_default_instance_;
template <typename>
PROTOBUF_CONSTEXPR FailureResult::FailureResult(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.result_str_)*/ {
    &::_pbi::fixed_address_empty_string, ::_pbi::ConstantInitialized {}
  }

  , /*decltype(_impl_.result_)*/ 0

  , /*decltype(_impl_._cached_size_)*/{}} {}
struct FailureResultDefaultTypeInternal {
  PROTOBUF_CONSTEXPR FailureResultDefaultTypeInternal() : _instance(::_pbi::ConstantInitialized{}) {}
  ~FailureResultDefaultTypeInternal() {}
  union {
    FailureResult _instance;
  };
};

PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT
    PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 FailureResultDefaultTypeInternal _FailureResult_default_instance_;
}  // namespace failure
}  // namespace rpc
}  // namespace mavsdk
static ::_pb::Metadata file_level_metadata_failure_2ffailure_2eproto[3];
static const ::_pb::EnumDescriptor* file_level_enum_descriptors_failure_2ffailure_2eproto[3];
static constexpr const ::_pb::ServiceDescriptor**
    file_level_service_descriptors_failure_2ffailure_2eproto = nullptr;
const ::uint32_t TableStruct_failure_2ffailure_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(
    protodesc_cold) = {
    ~0u,  // no _has_bits_
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::failure::InjectRequest, _internal_metadata_),
    ~0u,  // no _extensions_
    ~0u,  // no _oneof_case_
    ~0u,  // no _weak_field_map_
    ~0u,  // no _inlined_string_donated_
    ~0u,  // no _split_
    ~0u,  // no sizeof(Split)
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::failure::InjectRequest, _impl_.failure_unit_),
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::failure::InjectRequest, _impl_.failure_type_),
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::failure::InjectRequest, _impl_.instance_),
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::failure::InjectResponse, _impl_._has_bits_),
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::failure::InjectResponse, _internal_metadata_),
    ~0u,  // no _extensions_
    ~0u,  // no _oneof_case_
    ~0u,  // no _weak_field_map_
    ~0u,  // no _inlined_string_donated_
    ~0u,  // no _split_
    ~0u,  // no sizeof(Split)
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::failure::InjectResponse, _impl_.failure_result_),
    0,
    ~0u,  // no _has_bits_
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::failure::FailureResult, _internal_metadata_),
    ~0u,  // no _extensions_
    ~0u,  // no _oneof_case_
    ~0u,  // no _weak_field_map_
    ~0u,  // no _inlined_string_donated_
    ~0u,  // no _split_
    ~0u,  // no sizeof(Split)
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::failure::FailureResult, _impl_.result_),
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::failure::FailureResult, _impl_.result_str_),
};

static const ::_pbi::MigrationSchema
    schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
        { 0, -1, -1, sizeof(::mavsdk::rpc::failure::InjectRequest)},
        { 11, 20, -1, sizeof(::mavsdk::rpc::failure::InjectResponse)},
        { 21, -1, -1, sizeof(::mavsdk::rpc::failure::FailureResult)},
};

static const ::_pb::Message* const file_default_instances[] = {
    &::mavsdk::rpc::failure::_InjectRequest_default_instance_._instance,
    &::mavsdk::rpc::failure::_InjectResponse_default_instance_._instance,
    &::mavsdk::rpc::failure::_FailureResult_default_instance_._instance,
};
const char descriptor_table_protodef_failure_2ffailure_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
    "\n\025failure/failure.proto\022\022mavsdk.rpc.fail"
    "ure\032\024mavsdk_options.proto\"\217\001\n\rInjectRequ"
    "est\0225\n\014failure_unit\030\001 \001(\0162\037.mavsdk.rpc.f"
    "ailure.FailureUnit\0225\n\014failure_type\030\002 \001(\016"
    "2\037.mavsdk.rpc.failure.FailureType\022\020\n\010ins"
    "tance\030\003 \001(\005\"K\n\016InjectResponse\0229\n\016failure"
    "_result\030\001 \001(\0132!.mavsdk.rpc.failure.Failu"
    "reResult\"\227\002\n\rFailureResult\0228\n\006result\030\001 \001"
    "(\0162(.mavsdk.rpc.failure.FailureResult.Re"
    "sult\022\022\n\nresult_str\030\002 \001(\t\"\267\001\n\006Result\022\022\n\016R"
    "ESULT_UNKNOWN\020\000\022\022\n\016RESULT_SUCCESS\020\001\022\024\n\020R"
    "ESULT_NO_SYSTEM\020\002\022\033\n\027RESULT_CONNECTION_E"
    "RROR\020\003\022\026\n\022RESULT_UNSUPPORTED\020\004\022\021\n\rRESULT"
    "_DENIED\020\005\022\023\n\017RESULT_DISABLED\020\006\022\022\n\016RESULT"
    "_TIMEOUT\020\007*\375\003\n\013FailureUnit\022\034\n\030FAILURE_UN"
    "IT_SENSOR_GYRO\020\000\022\035\n\031FAILURE_UNIT_SENSOR_"
    "ACCEL\020\001\022\033\n\027FAILURE_UNIT_SENSOR_MAG\020\002\022\034\n\030"
    "FAILURE_UNIT_SENSOR_BARO\020\003\022\033\n\027FAILURE_UN"
    "IT_SENSOR_GPS\020\004\022$\n FAILURE_UNIT_SENSOR_O"
    "PTICAL_FLOW\020\005\022\033\n\027FAILURE_UNIT_SENSOR_VIO"
    "\020\006\022\'\n#FAILURE_UNIT_SENSOR_DISTANCE_SENSO"
    "R\020\007\022 \n\034FAILURE_UNIT_SENSOR_AIRSPEED\020\010\022\037\n"
    "\033FAILURE_UNIT_SYSTEM_BATTERY\020d\022\035\n\031FAILUR"
    "E_UNIT_SYSTEM_MOTOR\020e\022\035\n\031FAILURE_UNIT_SY"
    "STEM_SERVO\020f\022!\n\035FAILURE_UNIT_SYSTEM_AVOI"
    "DANCE\020g\022!\n\035FAILURE_UNIT_SYSTEM_RC_SIGNAL"
    "\020h\022&\n\"FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL"
    "\020i*\322\001\n\013FailureType\022\023\n\017FAILURE_TYPE_OK\020\000\022"
    "\024\n\020FAILURE_TYPE_OFF\020\001\022\026\n\022FAILURE_TYPE_ST"
    "UCK\020\002\022\030\n\024FAILURE_TYPE_GARBAGE\020\003\022\026\n\022FAILU"
    "RE_TYPE_WRONG\020\004\022\025\n\021FAILURE_TYPE_SLOW\020\005\022\030"
    "\n\024FAILURE_TYPE_DELAYED\020\006\022\035\n\031FAILURE_TYPE"
    "_INTERMITTENT\020\0072g\n\016FailureService\022U\n\006Inj"
    "ect\022!.mavsdk.rpc.failure.InjectRequest\032\""
    ".mavsdk.rpc.failure.InjectResponse\"\004\200\265\030\001"
    "B!\n\021io.mavsdk.failureB\014FailureProtob\006pro"
    "to3"
};
static const ::_pbi::DescriptorTable* const descriptor_table_failure_2ffailure_2eproto_deps[1] =
    {
        &::descriptor_table_mavsdk_5foptions_2eproto,
};
static ::absl::once_flag descriptor_table_failure_2ffailure_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_failure_2ffailure_2eproto = {
    false,
    false,
    1443,
    descriptor_table_protodef_failure_2ffailure_2eproto,
    "failure/failure.proto",
    &descriptor_table_failure_2ffailure_2eproto_once,
    descriptor_table_failure_2ffailure_2eproto_deps,
    1,
    3,
    schemas,
    file_default_instances,
    TableStruct_failure_2ffailure_2eproto::offsets,
    file_level_metadata_failure_2ffailure_2eproto,
    file_level_enum_descriptors_failure_2ffailure_2eproto,
    file_level_service_descriptors_failure_2ffailure_2eproto,
};

// This function exists to be marked as weak.
// It can significantly speed up compilation by breaking up LLVM's SCC
// in the .pb.cc translation units. Large translation units see a
// reduction of more than 35% of walltime for optimized builds. Without
// the weak attribute all the messages in the file, including all the
// vtables and everything they use become part of the same SCC through
// a cycle like:
// GetMetadata -> descriptor table -> default instances ->
//   vtables -> GetMetadata
// By adding a weak function here we break the connection from the
// individual vtables back into the descriptor table.
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_failure_2ffailure_2eproto_getter() {
  return &descriptor_table_failure_2ffailure_2eproto;
}
// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2
static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_failure_2ffailure_2eproto(&descriptor_table_failure_2ffailure_2eproto);
namespace mavsdk {
namespace rpc {
namespace failure {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* FailureResult_Result_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_failure_2ffailure_2eproto);
  return file_level_enum_descriptors_failure_2ffailure_2eproto[0];
}
bool FailureResult_Result_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
      return true;
    default:
      return false;
  }
}
#if (__cplusplus < 201703) && \
  (!defined(_MSC_VER) || (_MSC_VER >= 1900 && _MSC_VER < 1912))

constexpr FailureResult_Result FailureResult::RESULT_UNKNOWN;
constexpr FailureResult_Result FailureResult::RESULT_SUCCESS;
constexpr FailureResult_Result FailureResult::RESULT_NO_SYSTEM;
constexpr FailureResult_Result FailureResult::RESULT_CONNECTION_ERROR;
constexpr FailureResult_Result FailureResult::RESULT_UNSUPPORTED;
constexpr FailureResult_Result FailureResult::RESULT_DENIED;
constexpr FailureResult_Result FailureResult::RESULT_DISABLED;
constexpr FailureResult_Result FailureResult::RESULT_TIMEOUT;
constexpr FailureResult_Result FailureResult::Result_MIN;
constexpr FailureResult_Result FailureResult::Result_MAX;
constexpr int FailureResult::Result_ARRAYSIZE;

#endif  // (__cplusplus < 201703) &&
        // (!defined(_MSC_VER) || (_MSC_VER >= 1900 && _MSC_VER < 1912))
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* FailureUnit_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_failure_2ffailure_2eproto);
  return file_level_enum_descriptors_failure_2ffailure_2eproto[1];
}
bool FailureUnit_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 100:
    case 101:
    case 102:
    case 103:
    case 104:
    case 105:
      return true;
    default:
      return false;
  }
}
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* FailureType_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_failure_2ffailure_2eproto);
  return file_level_enum_descriptors_failure_2ffailure_2eproto[2];
}
bool FailureType_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
      return true;
    default:
      return false;
  }
}
// ===================================================================

class InjectRequest::_Internal {
 public:
};

InjectRequest::InjectRequest(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor(arena);
  // @@protoc_insertion_point(arena_constructor:mavsdk.rpc.failure.InjectRequest)
}
InjectRequest::InjectRequest(const InjectRequest& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(), _impl_(from._impl_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(
      from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:mavsdk.rpc.failure.InjectRequest)
}

inline void InjectRequest::SharedCtor(::_pb::Arena* arena) {
  (void)arena;
  new (&_impl_) Impl_{
      decltype(_impl_.failure_unit_) { 0 }

    , decltype(_impl_.failure_type_) { 0 }

    , decltype(_impl_.instance_) { 0 }

    , /*decltype(_impl_._cached_size_)*/{}
  };
}

InjectRequest::~InjectRequest() {
  // @@protoc_insertion_point(destructor:mavsdk.rpc.failure.InjectRequest)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void InjectRequest::SharedDtor() {
  ABSL_DCHECK(GetArenaForAllocation() == nullptr);
}

void InjectRequest::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void InjectRequest::Clear() {
// @@protoc_insertion_point(message_clear_start:mavsdk.rpc.failure.InjectRequest)
  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&_impl_.failure_unit_, 0, static_cast<::size_t>(
      reinterpret_cast<char*>(&_impl_.instance_) -
      reinterpret_cast<char*>(&_impl_.failure_unit_)) + sizeof(_impl_.instance_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* InjectRequest::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .mavsdk.rpc.failure.FailureUnit failure_unit = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::uint8_t>(tag) == 8)) {
          ::int32_t val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
          _internal_set_failure_unit(static_cast<::mavsdk::rpc::failure::FailureUnit>(val));
        } else {
          goto handle_unusual;
        }
        continue;
      // .mavsdk.rpc.failure.FailureType failure_type = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::uint8_t>(tag) == 16)) {
          ::int32_t val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
          _internal_set_failure_type(static_cast<::mavsdk::rpc::failure::FailureType>(val));
        } else {
          goto handle_unusual;
        }
        continue;
      // int32 instance = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::uint8_t>(tag) == 24)) {
          _impl_.instance_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else {
          goto handle_unusual;
        }
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

::uint8_t* InjectRequest::_InternalSerialize(
    ::uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:mavsdk.rpc.failure.InjectRequest)
  ::uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .mavsdk.rpc.failure.FailureUnit failure_unit = 1;
  if (this->_internal_failure_unit() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteEnumToArray(
        1, this->_internal_failure_unit(), target);
  }

  // .mavsdk.rpc.failure.FailureType failure_type = 2;
  if (this->_internal_failure_type() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteEnumToArray(
        2, this->_internal_failure_type(), target);
  }

  // int32 instance = 3;
  if (this->_internal_instance() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteInt32ToArray(
        3, this->_internal_instance(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:mavsdk.rpc.failure.InjectRequest)
  return target;
}

::size_t InjectRequest::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:mavsdk.rpc.failure.InjectRequest)
  ::size_t total_size = 0;

  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .mavsdk.rpc.failure.FailureUnit failure_unit = 1;
  if (this->_internal_failure_unit() != 0) {
    total_size += 1 +
                  ::_pbi::WireFormatLite::EnumSize(this->_internal_failure_unit());
  }

  // .mavsdk.rpc.failure.FailureType failure_type = 2;
  if (this->_internal_failure_type() != 0) {
    total_size += 1 +
                  ::_pbi::WireFormatLite::EnumSize(this->_internal_failure_type());
  }

  // int32 instance = 3;
  if (this->_internal_instance() != 0) {
    total_size += ::_pbi::WireFormatLite::Int32SizePlusOne(
        this->_internal_instance());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData InjectRequest::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    InjectRequest::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*InjectRequest::GetClassData() const { return &_class_data_; }


void InjectRequest::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<InjectRequest*>(&to_msg);
  auto& from = static_cast<const InjectRequest&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:mavsdk.rpc.failure.InjectRequest)
  ABSL_DCHECK_NE(&from, _this);
  ::uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_failure_unit() != 0) {
    _this->_internal_set_failure_unit(from._internal_failure_unit());
  }
  if (from._internal_failure_type() != 0) {
    _this->_internal_set_failure_type(from._internal_failure_type());
  }
  if (from._internal_instance() != 0) {
    _this->_internal_set_instance(from._internal_instance());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void InjectRequest::CopyFrom(const InjectRequest& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:mavsdk.rpc.failure.InjectRequest)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool InjectRequest::IsInitialized() const {
  return true;
}

void InjectRequest::InternalSwap(InjectRequest* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(InjectRequest, _impl_.instance_)
      + sizeof(InjectRequest::_impl_.instance_)
      - PROTOBUF_FIELD_OFFSET(InjectRequest, _impl_.failure_unit_)>(
          reinterpret_cast<char*>(&_impl_.failure_unit_),
          reinterpret_cast<char*>(&other->_impl_.failure_unit_));
}

::PROTOBUF_NAMESPACE_ID::Metadata InjectRequest::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_failure_2ffailure_2eproto_getter, &descriptor_table_failure_2ffailure_2eproto_once,
      file_level_metadata_failure_2ffailure_2eproto[0]);
}
// ===================================================================

class InjectResponse::_Internal {
 public:
  using HasBits = decltype(std::declval<InjectResponse>()._impl_._has_bits_);
  static constexpr ::int32_t kHasBitsOffset =
    8 * PROTOBUF_FIELD_OFFSET(InjectResponse, _impl_._has_bits_);
  static const ::mavsdk::rpc::failure::FailureResult& failure_result(const InjectResponse* msg);
  static void set_has_failure_result(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

const ::mavsdk::rpc::failure::FailureResult&
InjectResponse::_Internal::failure_result(const InjectResponse* msg) {
  return *msg->_impl_.failure_result_;
}
InjectResponse::InjectResponse(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor(arena);
  // @@protoc_insertion_point(arena_constructor:mavsdk.rpc.failure.InjectResponse)
}
InjectResponse::InjectResponse(const InjectResponse& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  InjectResponse* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_._has_bits_){from._impl_._has_bits_}
    , /*decltype(_impl_._cached_size_)*/{}
    , decltype(_impl_.failure_result_){nullptr}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if ((from._impl_._has_bits_[0] & 0x00000001u) != 0) {
    _this->_impl_.failure_result_ = new ::mavsdk::rpc::failure::FailureResult(*from._impl_.failure_result_);
  }
  // @@protoc_insertion_point(copy_constructor:mavsdk.rpc.failure.InjectResponse)
}

inline void InjectResponse::SharedCtor(::_pb::Arena* arena) {
  (void)arena;
  new (&_impl_) Impl_{
      decltype(_impl_._has_bits_){}
    , /*decltype(_impl_._cached_size_)*/{}
    , decltype(_impl_.failure_result_){nullptr}
  };
}

InjectResponse::~InjectResponse() {
  // @@protoc_insertion_point(destructor:mavsdk.rpc.failure.InjectResponse)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void InjectResponse::SharedDtor() {
  ABSL_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete _impl_.failure_result_;
}

void InjectResponse::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void InjectResponse::Clear() {
// @@protoc_insertion_point(message_clear_start:mavsdk.rpc.failure.InjectResponse)
  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _impl_._has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    ABSL_DCHECK(_impl_.failure_result_ != nullptr);
    _impl_.failure_result_->Clear();
  }
  _impl_._has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* InjectResponse::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .mavsdk.rpc.failure.FailureResult failure_result = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_failure_result(), ptr);
          CHK_(ptr);
        } else {
          goto handle_unusual;
        }
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  _impl_._has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

::uint8_t* InjectResponse::_InternalSerialize(
    ::uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:mavsdk.rpc.failure.InjectResponse)
  ::uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _impl_._has_bits_[0];
  // .mavsdk.rpc.failure.FailureResult failure_result = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, _Internal::failure_result(this),
        _Internal::failure_result(this).GetCachedSize(), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:mavsdk.rpc.failure.InjectResponse)
  return target;
}

::size_t InjectResponse::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:mavsdk.rpc.failure.InjectResponse)
  ::size_t total_size = 0;

  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .mavsdk.rpc.failure.FailureResult failure_result = 1;
  cached_has_bits = _impl_._has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.failure_result_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData InjectResponse::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    InjectResponse::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*InjectResponse::GetClassData() const { return &_class_data_; }


void InjectResponse::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<InjectResponse*>(&to_msg);
  auto& from = static_cast<const InjectResponse&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:mavsdk.rpc.failure.InjectResponse)
  ABSL_DCHECK_NE(&from, _this);
  ::uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if ((from._impl_._has_bits_[0] & 0x00000001u) != 0) {
    _this->_internal_mutable_failure_result()->::mavsdk::rpc::failure::FailureResult::MergeFrom(
        from._internal_failure_result());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void InjectResponse::CopyFrom(const InjectResponse& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:mavsdk.rpc.failure.InjectResponse)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool InjectResponse::IsInitialized() const {
  return true;
}

void InjectResponse::InternalSwap(InjectResponse* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_impl_._has_bits_[0], other->_impl_._has_bits_[0]);
  swap(_impl_.failure_result_, other->_impl_.failure_result_);
}

::PROTOBUF_NAMESPACE_ID::Metadata InjectResponse::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_failure_2ffailure_2eproto_getter, &descriptor_table_failure_2ffailure_2eproto_once,
      file_level_metadata_failure_2ffailure_2eproto[1]);
}
// ===================================================================

class FailureResult::_Internal {
 public:
};

FailureResult::FailureResult(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor(arena);
  // @@protoc_insertion_point(arena_constructor:mavsdk.rpc.failure.FailureResult)
}
FailureResult::FailureResult(const FailureResult& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  FailureResult* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.result_str_) {}

    , decltype(_impl_.result_) {}

    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  _impl_.result_str_.InitDefault();
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
        _impl_.result_str_.Set("", GetArenaForAllocation());
  #endif  // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (!from._internal_result_str().empty()) {
    _this->_impl_.result_str_.Set(from._internal_result_str(), _this->GetArenaForAllocation());
  }
  _this->_impl_.result_ = from._impl_.result_;
  // @@protoc_insertion_point(copy_constructor:mavsdk.rpc.failure.FailureResult)
}

inline void FailureResult::SharedCtor(::_pb::Arena* arena) {
  (void)arena;
  new (&_impl_) Impl_{
      decltype(_impl_.result_str_) {}

    , decltype(_impl_.result_) { 0 }

    , /*decltype(_impl_._cached_size_)*/{}
  };
  _impl_.result_str_.InitDefault();
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
        _impl_.result_str_.Set("", GetArenaForAllocation());
  #endif  // PROTOBUF_FORCE_COPY_DEFAULT_STRING
}

FailureResult::~FailureResult() {
  // @@protoc_insertion_point(destructor:mavsdk.rpc.failure.FailureResult)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void FailureResult::SharedDtor() {
  ABSL_DCHECK(GetArenaForAllocation() == nullptr);
  _impl_.result_str_.Destroy();
}

void FailureResult::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void FailureResult::Clear() {
// @@protoc_insertion_point(message_clear_start:mavsdk.rpc.failure.FailureResult)
  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _impl_.result_str_.ClearToEmpty();
  _impl_.result_ = 0;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* FailureResult::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .mavsdk.rpc.failure.FailureResult.Result result = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::uint8_t>(tag) == 8)) {
          ::int32_t val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
          _internal_set_result(static_cast<::mavsdk::rpc::failure::FailureResult_Result>(val));
        } else {
          goto handle_unusual;
        }
        continue;
      // string result_str = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::uint8_t>(tag) == 18)) {
          auto str = _internal_mutable_result_str();
          ptr = ::_pbi::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(ptr);
          CHK_(::_pbi::VerifyUTF8(str, "mavsdk.rpc.failure.FailureResult.result_str"));
        } else {
          goto handle_unusual;
        }
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

::uint8_t* FailureResult::_InternalSerialize(
    ::uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:mavsdk.rpc.failure.FailureResult)
  ::uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .mavsdk.rpc.failure.FailureResult.Result result = 1;
  if (this->_internal_result() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteEnumToArray(
        1, this->_internal_result(), target);
  }

  // string result_str = 2;
  if (!this->_internal_result_str().empty()) {
    const std::string& _s = this->_internal_result_str();
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
        _s.data(), static_cast<int>(_s.length()), ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE, "mavsdk.rpc.failure.FailureResult.result_str");
    target = stream->WriteStringMaybeAliased(2, _s, target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:mavsdk.rpc.failure.FailureResult)
  return target;
}

::size_t FailureResult::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:mavsdk.rpc.failure.FailureResult)
  ::size_t total_size = 0;

  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // string result_str = 2;
  if (!this->_internal_result_str().empty()) {
    total_size += 1 + ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
                                    this->_internal_result_str());
  }

  // .mavsdk.rpc.failure.FailureResult.Result result = 1;
  if (this->_internal_result() != 0) {
    total_size += 1 +
                  ::_pbi::WireFormatLite::EnumSize(this->_internal_result());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData FailureResult::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    FailureResult::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*FailureResult::GetClassData() const { return &_class_data_; }


void FailureResult::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<FailureResult*>(&to_msg);
  auto& from = static_cast<const FailureResult&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:mavsdk.rpc.failure.FailureResult)
  ABSL_DCHECK_NE(&from, _this);
  ::uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (!from._internal_result_str().empty()) {
    _this->_internal_set_result_str(from._internal_result_str());
  }
  if (from._internal_result() != 0) {
    _this->_internal_set_result(from._internal_result());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void FailureResult::CopyFrom(const FailureResult& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:mavsdk.rpc.failure.FailureResult)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool FailureResult::IsInitialized() const {
  return true;
}

void FailureResult::InternalSwap(FailureResult* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::_pbi::ArenaStringPtr::InternalSwap(&_impl_.result_str_, lhs_arena,
                                       &other->_impl_.result_str_, rhs_arena);
  swap(_impl_.result_, other->_impl_.result_);
}

::PROTOBUF_NAMESPACE_ID::Metadata FailureResult::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_failure_2ffailure_2eproto_getter, &descriptor_table_failure_2ffailure_2eproto_once,
      file_level_metadata_failure_2ffailure_2eproto[2]);
}
// @@protoc_insertion_point(namespace_scope)
}  // namespace failure
}  // namespace rpc
}  // namespace mavsdk
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::mavsdk::rpc::failure::InjectRequest*
Arena::CreateMaybeMessage< ::mavsdk::rpc::failure::InjectRequest >(Arena* arena) {
  return Arena::CreateMessageInternal< ::mavsdk::rpc::failure::InjectRequest >(arena);
}
template<> PROTOBUF_NOINLINE ::mavsdk::rpc::failure::InjectResponse*
Arena::CreateMaybeMessage< ::mavsdk::rpc::failure::InjectResponse >(Arena* arena) {
  return Arena::CreateMessageInternal< ::mavsdk::rpc::failure::InjectResponse >(arena);
}
template<> PROTOBUF_NOINLINE ::mavsdk::rpc::failure::FailureResult*
Arena::CreateMaybeMessage< ::mavsdk::rpc::failure::FailureResult >(Arena* arena) {
  return Arena::CreateMessageInternal< ::mavsdk::rpc::failure::FailureResult >(arena);
}
PROTOBUF_NAMESPACE_CLOSE
// @@protoc_insertion_point(global_scope)
#include "google/protobuf/port_undef.inc"
