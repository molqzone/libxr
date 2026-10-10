#include "object_dictionary.hpp"

namespace LibXR::EtherCAT
{

Object* ObjectDictionary::FindObject(uint16_t index)
{
  for (size_t object_index = 0; object_index < object_count_; ++object_index)
  {
    if (objects_[object_index].index == index)
    {
      return &objects_[object_index];
    }
  }
  return nullptr;
}

const Object* ObjectDictionary::FindObject(uint16_t index) const
{
  for (size_t object_index = 0; object_index < object_count_; ++object_index)
  {
    if (objects_[object_index].index == index)
    {
      return &objects_[object_index];
    }
  }
  return nullptr;
}

ObjectEntry* ObjectDictionary::FindEntry(ObjectAddress address)
{
  Object* object = FindObject(address.index);
  if (object == nullptr)
  {
    return nullptr;
  }

  for (size_t entry_index = 0; entry_index < object->entry_count; ++entry_index)
  {
    ObjectEntry& entry = const_cast<ObjectEntry&>(object->entries[entry_index]);
    if (entry.address.subindex == address.subindex)
    {
      return &entry;
    }
  }
  return nullptr;
}

const ObjectEntry* ObjectDictionary::FindEntry(ObjectAddress address) const
{
  const Object* object = FindObject(address.index);
  if (object == nullptr)
  {
    return nullptr;
  }

  for (size_t entry_index = 0; entry_index < object->entry_count; ++entry_index)
  {
    const ObjectEntry& entry = object->entries[entry_index];
    if (entry.address.subindex == address.subindex)
    {
      return &entry;
    }
  }
  return nullptr;
}

}  // namespace LibXR::EtherCAT
