#include "device_composition.hpp"

#include <cstring>
#include <limits>

namespace LibXR::EtherCAT
{

namespace
{

constexpr size_t BytesForBits(size_t bit_count) { return (bit_count + 7U) / 8U; }

void CopyBitsToProcessData(uint8_t* destination, size_t destination_bit_offset,
                           const uint8_t* source, size_t bit_count)
{
  for (size_t bit = 0; bit < bit_count; ++bit)
  {
    const uint8_t source_bit = static_cast<uint8_t>((source[bit / 8U] >> (bit % 8U)) & 1U);
    uint8_t& destination_byte = destination[(destination_bit_offset + bit) / 8U];
    const uint8_t destination_mask =
        static_cast<uint8_t>(1U << ((destination_bit_offset + bit) % 8U));
    destination_byte = source_bit != 0U ? static_cast<uint8_t>(destination_byte | destination_mask)
                                        : static_cast<uint8_t>(destination_byte & ~destination_mask);
  }
}

void CopyBitsFromProcessData(uint8_t* destination, const uint8_t* source,
                             size_t source_bit_offset, size_t bit_count)
{
  for (size_t bit = 0; bit < bit_count; ++bit)
  {
    const uint8_t source_bit = static_cast<uint8_t>(
        (source[(source_bit_offset + bit) / 8U] >> ((source_bit_offset + bit) % 8U)) & 1U);
    uint8_t& destination_byte = destination[bit / 8U];
    const uint8_t destination_mask = static_cast<uint8_t>(1U << (bit % 8U));
    destination_byte = source_bit != 0U ? static_cast<uint8_t>(destination_byte | destination_mask)
                                        : static_cast<uint8_t>(destination_byte & ~destination_mask);
  }
}

}  // namespace

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

size_t DeviceComposition::GetPdoByteSize(PdoDirection direction) const
{
  size_t bit_count = 0;
  for (size_t pdo_index = 0; pdo_index < pool_.pdo_count_; ++pdo_index)
  {
    const Pdo& pdo = pool_.storage_.pdos[pdo_index];
    if (pdo.direction == direction)
    {
      bit_count += pdo.bit_length;
    }
  }
  return BytesForBits(bit_count);
}

ErrorCode DeviceComposition::PackPdos(RawData process_data)
{
  const size_t process_data_size = GetPdoByteSize(PdoDirection::TX);
  if (process_data.addr_ == nullptr && process_data_size != 0U)
  {
    return ErrorCode::PTR_NULL;
  }
  if (process_data.size_ < process_data_size)
  {
    return ErrorCode::SIZE_ERR;
  }

  auto* bytes = static_cast<uint8_t*>(process_data.addr_);
  if (process_data_size != 0U)
  {
    std::memset(bytes, 0, process_data_size);
  }

  size_t pdo_bit_offset = 0;
  for (size_t pdo_index = 0; pdo_index < pool_.pdo_count_; ++pdo_index)
  {
    const Pdo& pdo = pool_.storage_.pdos[pdo_index];
    if (pdo.direction != PdoDirection::TX)
    {
      continue;
    }

    for (size_t entry_index = 0; entry_index < pdo.entry_count; ++entry_index)
    {
      const PdoEntry& pdo_entry = pdo.entries[entry_index];
      ObjectEntry& entry = *pdo_entry.object;
      if (entry.storage.addr_ == nullptr || entry.storage.size_ < BytesForBits(entry.bit_length))
      {
        return ErrorCode::PTR_NULL;
      }

      const ErrorCode result = DispatchObjectRead(entry.address);
      if (result != ErrorCode::OK)
      {
        return result;
      }

      CopyBitsToProcessData(bytes, pdo_bit_offset + pdo_entry.bit_offset,
                            static_cast<const uint8_t*>(entry.storage.addr_), entry.bit_length);
    }
    pdo_bit_offset += pdo.bit_length;
  }
  return ErrorCode::OK;
}

ErrorCode DeviceComposition::UnpackPdos(ConstRawData process_data)
{
  const size_t process_data_size = GetPdoByteSize(PdoDirection::RX);
  if (process_data.addr_ == nullptr && process_data_size != 0U)
  {
    return ErrorCode::PTR_NULL;
  }
  if (process_data.size_ < process_data_size)
  {
    return ErrorCode::SIZE_ERR;
  }

  const auto* bytes = static_cast<const uint8_t*>(process_data.addr_);
  size_t pdo_bit_offset = 0;
  for (size_t pdo_index = 0; pdo_index < pool_.pdo_count_; ++pdo_index)
  {
    const Pdo& pdo = pool_.storage_.pdos[pdo_index];
    if (pdo.direction != PdoDirection::RX)
    {
      continue;
    }

    for (size_t entry_index = 0; entry_index < pdo.entry_count; ++entry_index)
    {
      const PdoEntry& pdo_entry = pdo.entries[entry_index];
      ObjectEntry& entry = *pdo_entry.object;
      if (entry.storage.addr_ == nullptr || entry.storage.size_ < BytesForBits(entry.bit_length))
      {
        return ErrorCode::PTR_NULL;
      }

      CopyBitsFromProcessData(static_cast<uint8_t*>(entry.storage.addr_), bytes,
                              pdo_bit_offset + pdo_entry.bit_offset, entry.bit_length);
      const ErrorCode result = DispatchObjectWrite(entry.address);
      if (result != ErrorCode::OK)
      {
        return result;
      }
    }
    pdo_bit_offset += pdo.bit_length;
  }
  return ErrorCode::OK;
}

}  // namespace LibXR::EtherCAT
