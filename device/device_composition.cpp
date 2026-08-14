#include "device_composition.hpp"

#include <limits>

namespace LibXR::EtherCAT
{

Object& DeviceBuilder::AddObject(uint16_t index, ObjectCode code, const char* name)
{
  ASSERT(name != nullptr);

  for (size_t object_index = 0; object_index < pool_.object_count_; ++object_index)
  {
    ASSERT(pool_.storage_.objects[object_index].index != index);
  }

  Object& object = pool_.AddObject();
  object = {index, code, name, nullptr, 0, &owner_};
  open_object_ = &object;
  open_pdo_ = nullptr;
  return object;
}

ObjectEntry& DeviceBuilder::AddEntry(Object& object, uint8_t subindex, ObjectDataType type,
                                    uint16_t bit_length, ObjectAccess access, const char* name,
                                    RawData storage)
{
  ASSERT(&object == open_object_);
  ASSERT(object.owner == &owner_);
  ASSERT(name != nullptr);
  ASSERT(bit_length > 0);
  ASSERT(storage.addr_ != nullptr || storage.size_ == 0);
  ASSERT(storage.size_ == 0 || storage.size_ * 8U >= bit_length);

  for (size_t entry_index = 0; entry_index < object.entry_count; ++entry_index)
  {
    ASSERT(object.entries[entry_index].address.subindex != subindex);
  }

  if (object.entries == nullptr)
  {
    ASSERT(pool_.entry_count_ < pool_.storage_.entry_capacity);
    object.entries = &pool_.storage_.entries[pool_.entry_count_];
  }

  ObjectEntry& entry = pool_.AddEntry();
  entry = {{object.index, subindex}, type, bit_length, access, name, storage, &owner_};
  ++object.entry_count;
  return entry;
}

Pdo& DeviceBuilder::AddPdo(PdoDirection direction, uint16_t index)
{
  for (size_t pdo_index = 0; pdo_index < pool_.pdo_count_; ++pdo_index)
  {
    ASSERT(pool_.storage_.pdos[pdo_index].index != index);
  }

  Pdo& pdo = pool_.AddPdo();
  pdo = {index, direction, nullptr, 0, 0, &owner_};
  open_object_ = nullptr;
  open_pdo_ = &pdo;
  return pdo;
}

PdoEntry& DeviceBuilder::Map(Pdo& pdo, ObjectEntry& entry)
{
  ASSERT(&pdo == open_pdo_);
  ASSERT(pdo.owner == &owner_);
  ASSERT(entry.owner == &owner_);

  const ObjectAccess required_access =
      (pdo.direction == PdoDirection::RX) ? ObjectAccess::RX_PDO : ObjectAccess::TX_PDO;
  ASSERT(HasAccess(entry.access, required_access));

  const uint32_t next_bit_length = static_cast<uint32_t>(pdo.bit_length) + entry.bit_length;
  ASSERT(next_bit_length <= std::numeric_limits<uint16_t>::max());

  if (pdo.entries == nullptr)
  {
    ASSERT(pool_.pdo_entry_count_ < pool_.storage_.pdo_entry_capacity);
    pdo.entries = &pool_.storage_.pdo_entries[pool_.pdo_entry_count_];
  }

  PdoEntry& pdo_entry = pool_.AddPdoEntry();
  pdo_entry = {&entry, pdo.bit_length};
  ++pdo.entry_count;
  pdo.bit_length = static_cast<uint16_t>(next_bit_length);
  return pdo_entry;
}

DeviceComposition::DeviceComposition(DevicePool& pool,
                                   std::initializer_list<DeviceClass*> classes)
    : pool_(pool)
{
  ASSERT(pool_.Empty());

  for (DeviceClass* device_class : classes)
  {
    ASSERT(device_class != nullptr);
    pool_.AddClass(*device_class);
  }

  for (size_t class_index = 0; class_index < pool_.class_count_; ++class_index)
  {
    DeviceClass& device_class = *pool_.storage_.classes[class_index];
    DeviceBuilder builder(pool_, device_class);
    device_class.Describe(builder);
  }

  dictionary_ = ObjectDictionary(pool_.storage_.objects, pool_.object_count_);
}

const Pdo* DeviceComposition::FindPdo(PdoDirection direction, uint16_t index) const
{
  for (size_t pdo_index = 0; pdo_index < pool_.pdo_count_; ++pdo_index)
  {
    const Pdo& pdo = pool_.storage_.pdos[pdo_index];
    if (pdo.direction == direction && pdo.index == index)
    {
      return &pdo;
    }
  }
  return nullptr;
}

void DeviceComposition::DispatchStateChanged(AlState from, AlState to)
{
  for (size_t class_index = 0; class_index < pool_.class_count_; ++class_index)
  {
    pool_.storage_.classes[class_index]->OnStateChanged(from, to);
  }
}

void DeviceComposition::DispatchOutputsUpdated()
{
  for (size_t class_index = 0; class_index < pool_.class_count_; ++class_index)
  {
    pool_.storage_.classes[class_index]->OnOutputsUpdated();
  }
}

void DeviceComposition::DispatchInputsRequested()
{
  for (size_t class_index = 0; class_index < pool_.class_count_; ++class_index)
  {
    pool_.storage_.classes[class_index]->OnInputsRequested();
  }
}

ErrorCode DeviceComposition::DispatchObjectRead(ObjectAddress address)
{
  ObjectEntry* entry = dictionary_.FindEntry(address);
  return entry == nullptr ? ErrorCode::NOT_FOUND : entry->owner->OnObjectRead(*entry);
}

ErrorCode DeviceComposition::DispatchObjectWrite(ObjectAddress address)
{
  ObjectEntry* entry = dictionary_.FindEntry(address);
  return entry == nullptr ? ErrorCode::NOT_FOUND : entry->owner->OnObjectWrite(*entry);
}

}  // namespace LibXR::EtherCAT
