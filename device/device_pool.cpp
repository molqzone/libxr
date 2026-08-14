#include "device_pool.hpp"

namespace LibXR::EtherCAT
{

DevicePool::DevicePool(const Storage& storage) : storage_(storage)
{
  ASSERT(storage_.classes != nullptr && storage_.class_capacity > 0);
  ASSERT(storage_.objects != nullptr && storage_.object_capacity > 0);
  ASSERT(storage_.entries != nullptr && storage_.entry_capacity > 0);
  ASSERT(storage_.pdos != nullptr && storage_.pdo_capacity > 0);
  ASSERT(storage_.pdo_entries != nullptr && storage_.pdo_entry_capacity > 0);
}

bool DevicePool::Empty() const
{
  return class_count_ == 0 && object_count_ == 0 && entry_count_ == 0 && pdo_count_ == 0 &&
         pdo_entry_count_ == 0;
}

void DevicePool::AddClass(DeviceClass& device)
{
  ASSERT(class_count_ < storage_.class_capacity);
  storage_.classes[class_count_++] = &device;
}

Object& DevicePool::AddObject()
{
  ASSERT(object_count_ < storage_.object_capacity);
  return storage_.objects[object_count_++];
}

ObjectEntry& DevicePool::AddEntry()
{
  ASSERT(entry_count_ < storage_.entry_capacity);
  return storage_.entries[entry_count_++];
}

Pdo& DevicePool::AddPdo()
{
  ASSERT(pdo_count_ < storage_.pdo_capacity);
  return storage_.pdos[pdo_count_++];
}

PdoEntry& DevicePool::AddPdoEntry()
{
  ASSERT(pdo_entry_count_ < storage_.pdo_entry_capacity);
  return storage_.pdo_entries[pdo_entry_count_++];
}

}  // namespace LibXR::EtherCAT
