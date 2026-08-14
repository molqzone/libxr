#pragma once

#include <initializer_list>

#include "device_class.hpp"
#include "device_pool.hpp"

namespace LibXR::EtherCAT
{

/** Builder used by one DeviceClass during composition. */
class DeviceBuilder
{
 public:
  Object& AddObject(uint16_t index, ObjectCode code, const char* name);

  ObjectEntry& AddEntry(Object& object, uint8_t subindex, ObjectDataType type,
                        uint16_t bit_length, ObjectAccess access, const char* name,
                        RawData storage = {});

  template <typename Value>
  ObjectEntry& AddEntry(Object& object, uint8_t subindex, ObjectAccess access,
                        const char* name, Value& value)
  {
    return AddEntry(object, subindex, ObjectDataTypeOf<Value>(),
                    static_cast<uint16_t>(sizeof(Value) * 8U), access, name, RawData(value));
  }

  Pdo& AddPdo(PdoDirection direction, uint16_t index);
  PdoEntry& Map(Pdo& pdo, ObjectEntry& entry);

 private:
  friend class DeviceComposition;

  DeviceBuilder(DevicePool& pool, DeviceClass& owner) : pool_(pool), owner_(owner) {}

  DevicePool& pool_;
  DeviceClass& owner_;
  Object* open_object_ = nullptr;
  Pdo* open_pdo_ = nullptr;
};

/**
 * Immutable composition of all functional modules in one EtherCAT slave.
 *
 * It keeps non-owning references to DeviceClass modules in DevicePool storage,
 * while the ObjectDictionary and PDO descriptors are built from their
 * declarations during construction.
 */
class DeviceComposition
{
 public:
  DeviceComposition(DevicePool& pool, std::initializer_list<DeviceClass*> classes);

  DeviceComposition(const DeviceComposition&) = delete;
  DeviceComposition& operator=(const DeviceComposition&) = delete;
  DeviceComposition(DeviceComposition&&) = delete;
  DeviceComposition& operator=(DeviceComposition&&) = delete;

  [[nodiscard]] const ObjectDictionary& GetObjectDictionary() const { return dictionary_; }
  [[nodiscard]] const Pdo* FindPdo(PdoDirection direction, uint16_t index) const;

 private:
  friend class DeviceCore;

  void DispatchStateChanged(AlState from, AlState to);
  void DispatchOutputsUpdated();
  void DispatchInputsRequested();
  [[nodiscard]] ErrorCode DispatchObjectRead(ObjectAddress address);
  [[nodiscard]] ErrorCode DispatchObjectWrite(ObjectAddress address);

  DevicePool& pool_;
  ObjectDictionary dictionary_{};
};

}  // namespace LibXR::EtherCAT
