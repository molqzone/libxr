#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "core/libxr_type.hpp"

namespace LibXR::EtherCAT
{

class DeviceClass;

struct ObjectAddress
{
  uint16_t index = 0;
  uint8_t subindex = 0;

  [[nodiscard]] constexpr bool operator==(const ObjectAddress&) const = default;
};

enum class ObjectCode : uint8_t
{
  VARIABLE,
  ARRAY,
  RECORD
};

enum class ObjectDataType : uint16_t
{
  BOOLEAN = 0x0001,
  INTEGER8 = 0x0002,
  INTEGER16 = 0x0003,
  INTEGER32 = 0x0004,
  UNSIGNED8 = 0x0005,
  UNSIGNED16 = 0x0006,
  UNSIGNED32 = 0x0007,
  REAL32 = 0x0008,
  VISIBLE_STRING = 0x0009,
  OCTET_STRING = 0x000A,
  INTEGER64 = 0x0015,
  UNSIGNED64 = 0x001B,
  REAL64 = 0x0011
};

enum class ObjectAccess : uint16_t
{
  NONE = 0,
  READ_PRE_OPERATIONAL = 1U << 0U,
  READ_SAFE_OPERATIONAL = 1U << 1U,
  READ_OPERATIONAL = 1U << 2U,
  WRITE_PRE_OPERATIONAL = 1U << 3U,
  WRITE_SAFE_OPERATIONAL = 1U << 4U,
  WRITE_OPERATIONAL = 1U << 5U,
  RX_PDO = 1U << 6U,
  TX_PDO = 1U << 7U,
  BACKUP = 1U << 8U,
  SETTING = 1U << 9U,

  READ = READ_PRE_OPERATIONAL | READ_SAFE_OPERATIONAL | READ_OPERATIONAL,
  WRITE = WRITE_PRE_OPERATIONAL | WRITE_SAFE_OPERATIONAL | WRITE_OPERATIONAL,
  READ_WRITE = READ | WRITE
};

constexpr ObjectAccess operator|(ObjectAccess left, ObjectAccess right)
{
  return static_cast<ObjectAccess>(static_cast<uint16_t>(left) |
                                    static_cast<uint16_t>(right));
}

constexpr ObjectAccess operator&(ObjectAccess left, ObjectAccess right)
{
  return static_cast<ObjectAccess>(static_cast<uint16_t>(left) &
                                    static_cast<uint16_t>(right));
}

constexpr bool HasAccess(ObjectAccess access, ObjectAccess required)
{
  return (static_cast<uint16_t>(access) & static_cast<uint16_t>(required)) ==
         static_cast<uint16_t>(required);
}

template <typename Value>
constexpr ObjectDataType ObjectDataTypeOf()
{
  using Type = std::remove_cv_t<Value>;

  if constexpr (std::is_same_v<Type, bool>)
  {
    return ObjectDataType::BOOLEAN;
  }
  else if constexpr (std::is_same_v<Type, int8_t>)
  {
    return ObjectDataType::INTEGER8;
  }
  else if constexpr (std::is_same_v<Type, int16_t>)
  {
    return ObjectDataType::INTEGER16;
  }
  else if constexpr (std::is_same_v<Type, int32_t>)
  {
    return ObjectDataType::INTEGER32;
  }
  else if constexpr (std::is_same_v<Type, int64_t>)
  {
    return ObjectDataType::INTEGER64;
  }
  else if constexpr (std::is_same_v<Type, uint8_t>)
  {
    return ObjectDataType::UNSIGNED8;
  }
  else if constexpr (std::is_same_v<Type, uint16_t>)
  {
    return ObjectDataType::UNSIGNED16;
  }
  else if constexpr (std::is_same_v<Type, uint32_t>)
  {
    return ObjectDataType::UNSIGNED32;
  }
  else if constexpr (std::is_same_v<Type, uint64_t>)
  {
    return ObjectDataType::UNSIGNED64;
  }
  else if constexpr (std::is_same_v<Type, float>)
  {
    return ObjectDataType::REAL32;
  }
  else if constexpr (std::is_same_v<Type, double>)
  {
    return ObjectDataType::REAL64;
  }
  else
  {
    static_assert(std::is_same_v<Type, void>, "Unsupported EtherCAT object value type");
  }
}

struct ObjectEntry
{
  ObjectAddress address{};
  ObjectDataType type = ObjectDataType::UNSIGNED8;
  uint16_t bit_length = 0;
  ObjectAccess access = ObjectAccess::NONE;
  const char* name = nullptr;
  RawData storage{};
  DeviceClass* owner = nullptr;
};

struct Object
{
  uint16_t index = 0;
  ObjectCode code = ObjectCode::VARIABLE;
  const char* name = nullptr;
  ObjectEntry* entries = nullptr;
  size_t entry_count = 0;
  DeviceClass* owner = nullptr;
};

/** A non-owning view of the completed CoE object dictionary. */
class ObjectDictionary
{
 public:
  ObjectDictionary() = default;
  ObjectDictionary(Object* objects, size_t object_count)
      : objects_(objects), object_count_(object_count)
  {
  }

  [[nodiscard]] size_t Size() const { return object_count_; }
  [[nodiscard]] const Object* Data() const { return objects_; }

  [[nodiscard]] Object* FindObject(uint16_t index) const;
  [[nodiscard]] ObjectEntry* FindEntry(ObjectAddress address) const;

 private:
  Object* objects_ = nullptr;
  size_t object_count_ = 0;
};

}  // namespace LibXR::EtherCAT
