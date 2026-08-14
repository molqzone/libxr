#pragma once

#include <initializer_list>

#include "slave_class.hpp"
#include "slave_pool.hpp"

namespace LibXR::EtherCAT
{

/** Builder used by one SlaveClass during composition. */
class SlaveBuilder
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
  friend class SlaveComposition;

  SlaveBuilder(SlavePool& pool, SlaveClass& owner) : pool_(pool), owner_(owner) {}

  SlavePool& pool_;
  SlaveClass& owner_;
  Object* open_object_ = nullptr;
  Pdo* open_pdo_ = nullptr;
};

/**
 * Immutable composition of all functional modules in one EtherCAT slave.
 *
 * It keeps non-owning references to SlaveClass modules in SlavePool storage,
 * while the ObjectDictionary and PDO descriptors are built from their
 * declarations during construction.
 */
class SlaveComposition
{
 public:
  SlaveComposition(SlavePool& pool, std::initializer_list<SlaveClass*> classes);

  SlaveComposition(const SlaveComposition&) = delete;
  SlaveComposition& operator=(const SlaveComposition&) = delete;
  SlaveComposition(SlaveComposition&&) = delete;
  SlaveComposition& operator=(SlaveComposition&&) = delete;

  [[nodiscard]] const ObjectDictionary& GetObjectDictionary() const { return dictionary_; }
  [[nodiscard]] const Pdo* FindPdo(PdoDirection direction, uint16_t index) const;

 private:
  friend class SlaveCore;

  void DispatchStateChanged(SlaveState from, SlaveState to);
  void DispatchOutputsUpdated();
  void DispatchInputsRequested();
  [[nodiscard]] ErrorCode DispatchObjectRead(ObjectAddress address);
  [[nodiscard]] ErrorCode DispatchObjectWrite(ObjectAddress address);

  SlavePool& pool_;
  ObjectDictionary dictionary_{};
};

}  // namespace LibXR::EtherCAT
