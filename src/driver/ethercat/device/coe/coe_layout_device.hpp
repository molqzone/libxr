#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "core/object_dictionary.hpp"
#include "core/pdo.hpp"
#include "ethercat/device/device_composition.hpp"
#include "libxr_def.hpp"

namespace LibXR::EtherCAT
{

/**
 * A CoE DeviceClass built from one compile-time layout table: it owns the object
 * dictionary entries and the PDO image the table declares, plus the storage the
 * process data is packed into and unpacked from. A concrete DeviceClass owns
 * runtime behavior by overriding the normal device hooks; this helper only
 * models the wire layout and storage.
 *
 * The layout table (`Layout`, e.g. the view of the `EcatLayout` namespace a
 * firmware's module binding exposes as one type) is a plain LibXR-free constexpr
 * table, so it also compiles on the host and an inconsistent layout is a build
 * error, not a slave the master refuses to bring to OP. Its shape is the
 * contract this template consumes:
 *
 *   - `Layout::Entry`: `index`, `subindex`, `bit_length`, `type`, `access`,
 *     `pdo_index`, `pdo_direction`, `object_name`;
 *   - `Layout::Pdo`: `index`, `direction`;
 *   - `Layout::kEntries` / `Layout::kPdos`: pointers to the two tables with
 *     `Layout::kEntryCount` / `Layout::kPdoCount` entries, in description
 *     order -- within a PDO the table order is the master's mapping order, and
 *     across PDOs the `kPdos` order is the process data image order, because
 *     DeviceComposition concatenates in AddPdo() order;
 *   - `Layout::kObjectCount`: how many distinct objects the table declares;
 *   - `Layout::DataType`, `Layout::Direction`, `Layout::ACCESS_RX`,
 *     `Layout::ACCESS_TX`: the table's duplicated wire encodings, asserted
 *     against LibXR's enums below so a table that disagrees about the encoding
 *     never reaches a board.
 */
template <typename Layout>
class CoeLayoutDevice : public DeviceClass
{
 public:
  /** Storage for each entry is sized to the widest value in the layout. */
  static constexpr size_t kMaxStorageBytes = []
  {
    size_t maximum = 0;
    for (size_t index = 0; index < Layout::kEntryCount; ++index)
    {
      const size_t bytes =
          (static_cast<size_t>(Layout::kEntries[index].bit_length) + 7U) / 8U;
      if (bytes > maximum)
      {
        maximum = bytes;
      }
    }
    return maximum;
  }();
  static_assert(kMaxStorageBytes > 0U);

  CoeLayoutDevice()
  {
    // Bind the table once, so Describe() is a direct transcription of it.
    for (size_t index = 0; index < Layout::kEntryCount; ++index)
    {
      runtime_[index].layout = &Layout::kEntries[index];
    }
  }

  CoeLayoutDevice(const CoeLayoutDevice&) = delete;
  CoeLayoutDevice& operator=(const CoeLayoutDevice&) = delete;
  CoeLayoutDevice(CoeLayoutDevice&&) = delete;
  CoeLayoutDevice& operator=(CoeLayoutDevice&&) = delete;

  /**
   * The storage of one declared object entry.
   *
   * @return a view sized to the entry's bit length.
   * @note `REQUIRE`s that the layout declares the entry, so a typo in a
   *       device profile cannot silently produce a value that never moves.
   */
  [[nodiscard]] LibXR::RawData Bind(uint16_t index, uint8_t subindex)
  {
    RuntimeEntry* runtime = FindRuntime(index, subindex);
    REQUIRE(runtime != nullptr);
    // Unreachable when the REQUIRE above fires (libxr_fatal_error does not
    // return), but a defined value keeps the failure path from being undefined
    // behaviour.
    if (runtime == nullptr)
    {
      return {};
    }
    return LibXR::RawData(runtime->storage.data(),
                          BytesForBits(runtime->layout->bit_length));
  }

  /**
   * The storage of one declared object entry, as a typed pointer.
   *
   * @note `REQUIRE`s that the entry exists and that both its CoE data type and
   *       bit length match `Value`, which turns a layout/profile disagreement
   *       into a boot-time failure instead of a wrong value on the wire.
   *       The storage is aligned for any scalar the dictionary declares.
   */
  template <typename Value>
  [[nodiscard]] Value* BindAs(uint16_t index, uint8_t subindex)
  {
    RuntimeEntry* runtime = FindRuntime(index, subindex);
    REQUIRE(runtime != nullptr);
    REQUIRE(static_cast<uint16_t>(runtime->layout->type) ==
            static_cast<uint16_t>(ObjectDataTypeOf<Value>()));
    REQUIRE(runtime->layout->bit_length == ObjectBitLengthOf<Value>());
    return reinterpret_cast<Value*>(runtime->storage.data());
  }

 protected:
  void Describe(DeviceBuilder& builder) override
  {
    // Objects and entries first, in table order: within a PDO the table order is
    // the master's mapping order, so the subindex sequence is part of the wire
    // contract.
    for (RuntimeEntry& runtime : runtime_)
    {
      const auto& layout = *runtime.layout;
      LibXR::EtherCAT::Object* object =
          FindOrAddObject(builder, layout.index, layout.object_name);
      runtime.entry = &builder.AddEntry(
          *object, layout.subindex, static_cast<ObjectDataType>(layout.type),
          layout.bit_length, static_cast<ObjectAccess>(layout.access), layout.object_name,
          LibXR::RawData(runtime.storage.data(), BytesForBits(layout.bit_length)));
    }

    // Then the PDOs in table order. That order is what decides the process data
    // layout, because DeviceComposition concatenates in AddPdo() order.
    for (size_t position = 0; position < Layout::kPdoCount; ++position)
    {
      const auto& layout = Layout::kPdos[position];
      REQUIRE(pdo_count_ < pdos_.size());
      auto& pdo = builder.AddPdo(
          layout.direction == Layout::Direction::RX ? PdoDirection::RX : PdoDirection::TX,
          layout.index);
      pdos_[pdo_count_++] = &pdo;

      for (RuntimeEntry& runtime : runtime_)
      {
        if (runtime.layout->pdo_index == layout.index)
        {
          builder.Map(pdo, *runtime.entry);
        }
      }
    }
  }

 private:
  // The table's wire encodings are duplicated on purpose (the table compiles
  // without LibXR); a mismatch means the generated table and the stack disagree
  // about what goes on the wire, which must never reach a board.
  static_assert(static_cast<uint16_t>(Layout::DataType::BOOLEAN) ==
                static_cast<uint16_t>(ObjectDataType::BOOLEAN));
  static_assert(static_cast<uint16_t>(Layout::DataType::INTEGER8) ==
                static_cast<uint16_t>(ObjectDataType::INTEGER8));
  static_assert(static_cast<uint16_t>(Layout::DataType::INTEGER16) ==
                static_cast<uint16_t>(ObjectDataType::INTEGER16));
  static_assert(static_cast<uint16_t>(Layout::DataType::INTEGER32) ==
                static_cast<uint16_t>(ObjectDataType::INTEGER32));
  static_assert(static_cast<uint16_t>(Layout::DataType::UNSIGNED8) ==
                static_cast<uint16_t>(ObjectDataType::UNSIGNED8));
  static_assert(static_cast<uint16_t>(Layout::DataType::UNSIGNED16) ==
                static_cast<uint16_t>(ObjectDataType::UNSIGNED16));
  static_assert(static_cast<uint16_t>(Layout::DataType::UNSIGNED32) ==
                static_cast<uint16_t>(ObjectDataType::UNSIGNED32));
  static_assert(static_cast<uint16_t>(Layout::DataType::REAL32) ==
                static_cast<uint16_t>(ObjectDataType::REAL32));
  static_assert(static_cast<uint16_t>(Layout::DataType::VISIBLE_STRING) ==
                static_cast<uint16_t>(ObjectDataType::VISIBLE_STRING));
  static_assert(static_cast<uint16_t>(Layout::DataType::OCTET_STRING) ==
                static_cast<uint16_t>(ObjectDataType::OCTET_STRING));
  static_assert(static_cast<uint16_t>(Layout::DataType::REAL64) ==
                static_cast<uint16_t>(ObjectDataType::REAL64));
  static_assert(static_cast<uint16_t>(Layout::DataType::INTEGER64) ==
                static_cast<uint16_t>(ObjectDataType::INTEGER64));
  static_assert(static_cast<uint16_t>(Layout::DataType::UNSIGNED64) ==
                static_cast<uint16_t>(ObjectDataType::UNSIGNED64));

  // ACCESS_RX = RX_PDO | WRITE, ACCESS_TX = TX_PDO | READ; the PDO direction bit
  // is what Map() requires, the read/write bits are what an SDO access checks.
  static_assert(static_cast<uint16_t>(Layout::ACCESS_RX) ==
                static_cast<uint16_t>(ObjectAccess::RX_PDO | ObjectAccess::WRITE));
  static_assert(static_cast<uint16_t>(Layout::ACCESS_TX) ==
                static_cast<uint16_t>(ObjectAccess::TX_PDO | ObjectAccess::READ));

  static_assert(static_cast<uint8_t>(Layout::Direction::RX) ==
                static_cast<uint8_t>(PdoDirection::RX));
  static_assert(static_cast<uint8_t>(Layout::Direction::TX) ==
                static_cast<uint8_t>(PdoDirection::TX));

  static_assert(Layout::kEntryCount > 0U);
  static_assert(Layout::kObjectCount > 0U);
  static_assert(Layout::kPdoCount > 0U);

  /** One table entry plus the storage DeviceCore packs and unpacks. */
  struct RuntimeEntry
  {
    const typename Layout::Entry* layout{};
    LibXR::EtherCAT::ObjectEntry* entry{};
    // The pack/unpack path only copies bytes, but a typed SDO upload presents
    // this address as the object's value, so it must be aligned for any scalar
    // the dictionary declares.
    alignas(std::max_align_t) std::array<uint8_t, kMaxStorageBytes> storage{};
  };

  static size_t BytesForBits(uint16_t bit_length)
  {
    return (static_cast<size_t>(bit_length) + 7U) / 8U;
  }

  RuntimeEntry* FindRuntime(uint16_t index, uint8_t subindex)
  {
    for (RuntimeEntry& runtime : runtime_)
    {
      if (runtime.layout->index == index && runtime.layout->subindex == subindex)
      {
        return &runtime;
      }
    }
    return nullptr;
  }

  LibXR::EtherCAT::Object* FindOrAddObject(DeviceBuilder& builder, uint16_t index,
                                           const char* name)
  {
    for (size_t position = 0; position < object_count_; ++position)
    {
      if (objects_[position]->index == index)
      {
        return objects_[position];
      }
    }

    REQUIRE(object_count_ < objects_.size());
    LibXR::EtherCAT::Object* object =
        &builder.AddObject(index, ObjectCode::VARIABLE, name);
    objects_[object_count_++] = object;
    return object;
  }

  std::array<RuntimeEntry, Layout::kEntryCount> runtime_{};
  // Describe() scratch, filled once while the composition is built and only
  // read afterwards.
  std::array<LibXR::EtherCAT::Object*, Layout::kObjectCount> objects_{};
  size_t object_count_{};
  std::array<LibXR::EtherCAT::Pdo*, Layout::kPdoCount> pdos_{};
  size_t pdo_count_{};
};

}  // namespace LibXR::EtherCAT
