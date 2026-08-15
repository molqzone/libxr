#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "core/object_dictionary.hpp"
#include "core/pdo.hpp"

namespace LibXR::EtherCAT
{

class DeviceBuilder;
class DeviceClass;
class DeviceComposition;

/**
 * Caller-owned storage used while composing one EtherCAT device.
 *
 * The pool only indexes fixed storage. It never allocates application data or
 * controls the lifetime of DeviceClass modules.
 */
class DevicePool
{
 public:
  struct Storage
  {
    DeviceClass** classes = nullptr;
    size_t class_capacity = 0;
    Object* objects = nullptr;
    size_t object_capacity = 0;
    ObjectEntry* entries = nullptr;
    size_t entry_capacity = 0;
    Pdo* pdos = nullptr;
    size_t pdo_capacity = 0;
    PdoEntry* pdo_entries = nullptr;
    size_t pdo_entry_capacity = 0;
    uint8_t* process_data = nullptr;
    size_t process_data_capacity = 0;
    uint8_t* mailbox = nullptr;
    size_t mailbox_capacity = 0;
  };

  explicit DevicePool(const Storage& storage);

  DevicePool(const DevicePool&) = delete;
  DevicePool& operator=(const DevicePool&) = delete;
  DevicePool(DevicePool&&) = delete;
  DevicePool& operator=(DevicePool&&) = delete;

  [[nodiscard]] size_t ClassCount() const { return class_count_; }
  [[nodiscard]] size_t ObjectCount() const { return object_count_; }
  [[nodiscard]] size_t EntryCount() const { return entry_count_; }
  [[nodiscard]] size_t PdoCount() const { return pdo_count_; }
  [[nodiscard]] size_t PdoEntryCount() const { return pdo_entry_count_; }

 private:
  friend class DeviceBuilder;
  friend class DeviceComposition;
  friend class DeviceCore;

  [[nodiscard]] bool Empty() const;
  void AddClass(DeviceClass& slave);
  Object& AddObject();
  ObjectEntry& AddEntry();
  Pdo& AddPdo();
  PdoEntry& AddPdoEntry();

  Storage storage_{};
  size_t class_count_ = 0;
  size_t object_count_ = 0;
  size_t entry_count_ = 0;
  size_t pdo_count_ = 0;
  size_t pdo_entry_count_ = 0;
};

namespace Detail
{

template <size_t ClassCapacity, size_t ObjectCapacity, size_t EntryCapacity,
          size_t PdoCapacity, size_t PdoEntryCapacity, size_t ProcessDataCapacity,
          size_t MailboxCapacity>
class StaticDevicePoolStorage
{
 protected:
  [[nodiscard]] DevicePool::Storage GetStorage()
  {
    return {classes_.data(), ClassCapacity, objects_.data(), ObjectCapacity,
            entries_.data(), EntryCapacity, pdos_.data(), PdoCapacity,
            pdo_entries_.data(), PdoEntryCapacity, process_data_.data(),
            ProcessDataCapacity, mailbox_.data(), MailboxCapacity};
  }

 private:
  std::array<DeviceClass*, ClassCapacity> classes_{};
  std::array<Object, ObjectCapacity> objects_{};
  std::array<ObjectEntry, EntryCapacity> entries_{};
  std::array<Pdo, PdoCapacity> pdos_{};
  std::array<PdoEntry, PdoEntryCapacity> pdo_entries_{};
  std::array<uint8_t, ProcessDataCapacity> process_data_{};
  std::array<uint8_t, MailboxCapacity> mailbox_{};
};

}  // namespace Detail

/**
 * Fixed-capacity DevicePool suitable for bare-metal applications.
 *
 * Capacity is declared alongside the application composition and no dynamic
 * allocation is used while the dictionary and PDO mapping are built.
 */
template <size_t ClassCapacity, size_t ObjectCapacity, size_t EntryCapacity,
          size_t PdoCapacity, size_t PdoEntryCapacity, size_t ProcessDataCapacity = 128,
          size_t MailboxCapacity = 128>
class StaticDevicePool final
    : private Detail::StaticDevicePoolStorage<ClassCapacity, ObjectCapacity, EntryCapacity,
                                              PdoCapacity, PdoEntryCapacity,
                                              ProcessDataCapacity, MailboxCapacity>,
      public DevicePool
{
 private:
  using Storage = Detail::StaticDevicePoolStorage<ClassCapacity, ObjectCapacity, EntryCapacity,
                                                   PdoCapacity, PdoEntryCapacity,
                                                   ProcessDataCapacity, MailboxCapacity>;

 public:
  static_assert(ClassCapacity > 0);
  static_assert(ObjectCapacity > 0);
  static_assert(EntryCapacity > 0);
  static_assert(PdoCapacity > 0);
  static_assert(PdoEntryCapacity > 0);
  static_assert(ProcessDataCapacity > 0);
  static_assert(MailboxCapacity > 0);

  StaticDevicePool() : Storage(), DevicePool(Storage::GetStorage()) {}
};

}  // namespace LibXR::EtherCAT
