#pragma once

#include <array>
#include <cstddef>

#include "core/object_dictionary.hpp"
#include "core/pdo.hpp"

namespace LibXR::EtherCAT
{

class SlaveBuilder;
class SlaveClass;
class SlaveComposition;

/**
 * Caller-owned storage used while composing one EtherCAT slave.
 *
 * The pool only indexes fixed storage. It never allocates application data or
 * controls the lifetime of SlaveClass modules.
 */
class SlavePool
{
 public:
  struct Storage
  {
    SlaveClass** classes = nullptr;
    size_t class_capacity = 0;
    Object* objects = nullptr;
    size_t object_capacity = 0;
    ObjectEntry* entries = nullptr;
    size_t entry_capacity = 0;
    Pdo* pdos = nullptr;
    size_t pdo_capacity = 0;
    PdoEntry* pdo_entries = nullptr;
    size_t pdo_entry_capacity = 0;
  };

  explicit SlavePool(const Storage& storage);

  SlavePool(const SlavePool&) = delete;
  SlavePool& operator=(const SlavePool&) = delete;
  SlavePool(SlavePool&&) = delete;
  SlavePool& operator=(SlavePool&&) = delete;

  [[nodiscard]] size_t ClassCount() const { return class_count_; }
  [[nodiscard]] size_t ObjectCount() const { return object_count_; }
  [[nodiscard]] size_t EntryCount() const { return entry_count_; }
  [[nodiscard]] size_t PdoCount() const { return pdo_count_; }
  [[nodiscard]] size_t PdoEntryCount() const { return pdo_entry_count_; }

 private:
  friend class SlaveBuilder;
  friend class SlaveComposition;

  [[nodiscard]] bool Empty() const;
  void AddClass(SlaveClass& slave);
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

/**
 * Fixed-capacity SlavePool suitable for bare-metal applications.
 *
 * Capacity is declared alongside the application composition and no dynamic
 * allocation is used while the dictionary and PDO mapping are built.
 */
template <size_t ClassCapacity, size_t ObjectCapacity, size_t EntryCapacity,
          size_t PdoCapacity, size_t PdoEntryCapacity>
class StaticSlavePool final : public SlavePool
{
 public:
  static_assert(ClassCapacity > 0);
  static_assert(ObjectCapacity > 0);
  static_assert(EntryCapacity > 0);
  static_assert(PdoCapacity > 0);
  static_assert(PdoEntryCapacity > 0);

  StaticSlavePool()
      : SlavePool({classes_.data(), ClassCapacity, objects_.data(), ObjectCapacity,
                   entries_.data(), EntryCapacity, pdos_.data(), PdoCapacity,
                   pdo_entries_.data(), PdoEntryCapacity})
  {
  }

 private:
  std::array<SlaveClass*, ClassCapacity> classes_{};
  std::array<Object, ObjectCapacity> objects_{};
  std::array<ObjectEntry, EntryCapacity> entries_{};
  std::array<Pdo, PdoCapacity> pdos_{};
  std::array<PdoEntry, PdoEntryCapacity> pdo_entries_{};
};

}  // namespace LibXR::EtherCAT
