#include <array>
#include <cassert>
#include <cstring>
#include <cstdint>
#include <initializer_list>

#include "device/device_core.hpp"

using namespace LibXR;
using namespace LibXR::EtherCAT;

namespace
{

class FakeEsc final : public EscPort
{
 public:
  ErrorCode Read(uint16_t address, RawData data) override
  {
    if (data.addr_ == nullptr && data.size_ != 0U)
    {
      return ErrorCode::PTR_NULL;
    }
    if (static_cast<size_t>(address) + data.size_ > memory.size())
    {
      return ErrorCode::OUT_OF_RANGE;
    }
    std::memcpy(data.addr_, memory.data() + address, data.size_);
    return ErrorCode::OK;
  }

  ErrorCode Write(uint16_t address, ConstRawData data) override
  {
    if (data.addr_ == nullptr && data.size_ != 0U)
    {
      return ErrorCode::PTR_NULL;
    }
    if (static_cast<size_t>(address) + data.size_ > memory.size())
    {
      return ErrorCode::OUT_OF_RANGE;
    }
    std::memcpy(memory.data() + address, data.addr_, data.size_);
    return ErrorCode::OK;
  }

  std::array<uint8_t, 0x10000> memory{};
};

void Put16(std::array<uint8_t, 0x10000>& memory, uint16_t address, uint16_t value)
{
  memory[address] = static_cast<uint8_t>(value);
  memory[address + 1U] = static_cast<uint8_t>(value >> 8U);
}

void Put32(std::array<uint8_t, 0x10000>& memory, uint16_t address, uint32_t value)
{
  Put16(memory, address, static_cast<uint16_t>(value));
  Put16(memory, static_cast<uint16_t>(address + 2U), static_cast<uint16_t>(value >> 16U));
}

class IoDevice final : public DeviceClass
{
 public:
  uint8_t output = 0;
  uint8_t input = 0;
  std::array<uint8_t, 8> parameter{};

 protected:
  void Describe(DeviceBuilder& builder) override
  {
    Object& output_object = builder.AddObject(0x6000, ObjectCode::VARIABLE, "Output");
    ObjectEntry& output_entry = builder.AddEntry(
        output_object, 0, ObjectDataType::UNSIGNED8, 8,
        ObjectAccess::WRITE | ObjectAccess::RX_PDO, "Output", RawData(output));
    Pdo& rx = builder.AddPdo(PdoDirection::RX, 0x1600);
    builder.Map(rx, output_entry);

    Object& input_object = builder.AddObject(0x6010, ObjectCode::VARIABLE, "Input");
    ObjectEntry& input_entry = builder.AddEntry(
        input_object, 0, ObjectDataType::UNSIGNED8, 8,
        ObjectAccess::READ | ObjectAccess::TX_PDO, "Input", RawData(input));
    Pdo& tx = builder.AddPdo(PdoDirection::TX, 0x1A00);
    builder.Map(tx, input_entry);

    Object& parameter_object = builder.AddObject(0x2000, ObjectCode::VARIABLE, "Parameter");
    builder.AddEntry(parameter_object, 0, ObjectDataType::OCTET_STRING, 64,
                     ObjectAccess::READ_WRITE, "Parameter",
                     RawData(parameter.data(), parameter.size()));
  }
};

}  // namespace

int main()
{
  FakeEsc esc;
  IoDevice application;
  StaticDevicePool<1, 3, 3, 2, 2, 8, 64> pool;

  Put16(esc.memory, 0x0800, 0x1000);
  Put16(esc.memory, 0x0802, 0);
  esc.memory[0x0804] = 0;
  Put16(esc.memory, 0x0808, 0x1000);
  Put16(esc.memory, 0x080A, 0);
  esc.memory[0x080C] = 0;

  Put16(esc.memory, 0x0810, 0x1000);
  Put16(esc.memory, 0x0812, 1);
  esc.memory[0x0814] = 0x04;
  Put16(esc.memory, 0x0818, 0x1100);
  Put16(esc.memory, 0x081A, 1);
  esc.memory[0x081C] = 0x00;

  Put32(esc.memory, 0x0600, 0x00000000);
  Put16(esc.memory, 0x0604, 1);
  Put16(esc.memory, 0x0608, 0x1000);
  esc.memory[0x060A] = 0x00;
  esc.memory[0x060B] = 0x02;
  esc.memory[0x060C] = 0x01;

  Put32(esc.memory, 0x0610, 0x00000000);
  Put16(esc.memory, 0x0614, 1);
  Put16(esc.memory, 0x0618, 0x1100);
  esc.memory[0x061A] = 0x00;
  esc.memory[0x061B] = 0x01;
  esc.memory[0x061C] = 0x01;

  DeviceCore core(esc, pool, {&application});

  Put16(esc.memory, 0x0120, static_cast<uint16_t>(AlState::PRE_OPERATIONAL));
  core.HandleInterrupt(EscEvent::AL_CONTROL);
  assert(core.GetState() == AlState::PRE_OPERATIONAL);

  Put16(esc.memory, 0x0120, static_cast<uint16_t>(AlState::SAFE_OPERATIONAL));
  core.HandleInterrupt(EscEvent::AL_CONTROL | EscEvent::SYNC_MANAGER_CHANGE);
  assert(core.GetState() == AlState::SAFE_OPERATIONAL);

  Put16(esc.memory, 0x0120, static_cast<uint16_t>(AlState::OPERATIONAL));
  core.HandleInterrupt(EscEvent::AL_CONTROL);
  assert(core.GetState() == AlState::OPERATIONAL);

  esc.memory[0x1000] = 0xA5;
  core.HandleInterrupt(EscEvent::PROCESS_DATA_OUTPUT);
  assert(application.output == 0xA5);

  application.input = 0x5A;
  core.HandleInterrupt(EscEvent::PROCESS_DATA_INPUT);
  assert(esc.memory[0x1100] == 0x5A);

  FakeEsc mailbox_esc;
  IoDevice mailbox_application;
  StaticDevicePool<1, 3, 3, 2, 2, 8, 64> mailbox_pool;
  Put16(mailbox_esc.memory, 0x0800, 0x1000);
  Put16(mailbox_esc.memory, 0x0802, 64);
  mailbox_esc.memory[0x0804] = 0x26;
  Put16(mailbox_esc.memory, 0x0808, 0x1040);
  Put16(mailbox_esc.memory, 0x080A, 64);
  mailbox_esc.memory[0x080C] = 0x22;

  DeviceCore mailbox_core(mailbox_esc, mailbox_pool, {&mailbox_application});
  Put16(mailbox_esc.memory, 0x0120, static_cast<uint16_t>(AlState::PRE_OPERATIONAL));
  mailbox_core.HandleInterrupt(EscEvent::AL_CONTROL);
  assert(mailbox_core.GetState() == AlState::PRE_OPERATIONAL);

  Put16(mailbox_esc.memory, 0x1000, 6);
  mailbox_esc.memory[0x1005] = 0x03;
  Put16(mailbox_esc.memory, 0x1006, 0x2000);
  mailbox_esc.memory[0x1008] = 0x40;
  Put16(mailbox_esc.memory, 0x1009, 0x6010);
  mailbox_esc.memory[0x100B] = 0;
  mailbox_application.input = 0x31;
  Put32(mailbox_esc.memory, 0x0220, 1U << 8U);
  mailbox_core.HandleInterrupt(EscEvent::MAILBOX);
  assert(mailbox_esc.memory[0x1040] == 10);
  assert(mailbox_esc.memory[0x1048] == 0x4F);
  assert(mailbox_esc.memory[0x104C] == 0x31);

  Put16(mailbox_esc.memory, 0x1000, 10);
  mailbox_esc.memory[0x1005] = 0x03;
  Put16(mailbox_esc.memory, 0x1006, 0x2000);
  mailbox_esc.memory[0x1008] = 0x2F;
  Put16(mailbox_esc.memory, 0x1009, 0x6000);
  mailbox_esc.memory[0x100B] = 0;
  mailbox_esc.memory[0x100C] = 0x77;
  Put32(mailbox_esc.memory, 0x0220, 1U << 8U);
  mailbox_core.HandleInterrupt(EscEvent::MAILBOX);
  assert(mailbox_application.output == 0x77);
  assert(mailbox_esc.memory[0x1040] == 6);
  assert(mailbox_esc.memory[0x1048] == 0x60);

  FakeEsc segmented_esc;
  IoDevice segmented_application;
  StaticDevicePool<1, 3, 3, 2, 2, 8, 16> segmented_pool;
  Put16(segmented_esc.memory, 0x0800, 0x1000);
  Put16(segmented_esc.memory, 0x0802, 16);
  segmented_esc.memory[0x0804] = 0x26;
  Put16(segmented_esc.memory, 0x0808, 0x1040);
  Put16(segmented_esc.memory, 0x080A, 16);
  segmented_esc.memory[0x080C] = 0x22;

  for (uint8_t index = 0; index < segmented_application.parameter.size(); ++index)
  {
    segmented_application.parameter[index] = static_cast<uint8_t>(index + 1U);
  }
  DeviceCore segmented_core(segmented_esc, segmented_pool, {&segmented_application});
  Put16(segmented_esc.memory, 0x0120, static_cast<uint16_t>(AlState::PRE_OPERATIONAL));
  segmented_core.HandleInterrupt(EscEvent::AL_CONTROL);

  Put16(segmented_esc.memory, 0x1000, 6);
  segmented_esc.memory[0x1005] = 0x03;
  Put16(segmented_esc.memory, 0x1006, 0x2000);
  segmented_esc.memory[0x1008] = 0x40;
  Put16(segmented_esc.memory, 0x1009, 0x2000);
  segmented_esc.memory[0x100B] = 0;
  Put32(segmented_esc.memory, 0x0220, 1U << 8U);
  segmented_core.HandleInterrupt(EscEvent::MAILBOX);
  assert(segmented_esc.memory[0x1048] == 0x41);

  Put16(segmented_esc.memory, 0x1000, 3);
  segmented_esc.memory[0x1005] = 0x03;
  Put16(segmented_esc.memory, 0x1006, 0x2000);
  segmented_esc.memory[0x1008] = 0x60;
  Put32(segmented_esc.memory, 0x0220, 1U << 8U);
  segmented_core.HandleInterrupt(EscEvent::MAILBOX);
  assert(segmented_esc.memory[0x1048] == 0x00);
  assert(segmented_esc.memory[0x1049] == 1);
  assert(segmented_esc.memory[0x104F] == 7);

  Put16(segmented_esc.memory, 0x1000, 3);
  segmented_esc.memory[0x1005] = 0x03;
  Put16(segmented_esc.memory, 0x1006, 0x2000);
  segmented_esc.memory[0x1008] = 0x70;
  Put32(segmented_esc.memory, 0x0220, 1U << 8U);
  segmented_core.HandleInterrupt(EscEvent::MAILBOX);
  assert(segmented_esc.memory[0x1048] == 0x1D);
  assert(segmented_esc.memory[0x1049] == 8);

  segmented_application.parameter.fill(0);
  Put16(segmented_esc.memory, 0x1000, 10);
  segmented_esc.memory[0x1005] = 0x03;
  Put16(segmented_esc.memory, 0x1006, 0x2000);
  segmented_esc.memory[0x1008] = 0x21;
  Put16(segmented_esc.memory, 0x1009, 0x2000);
  segmented_esc.memory[0x100B] = 0;
  Put32(segmented_esc.memory, 0x100C, 8);
  Put32(segmented_esc.memory, 0x0220, 1U << 8U);
  segmented_core.HandleInterrupt(EscEvent::MAILBOX);
  assert(segmented_esc.memory[0x1048] == 0x60);

  Put16(segmented_esc.memory, 0x1000, 10);
  segmented_esc.memory[0x1005] = 0x03;
  Put16(segmented_esc.memory, 0x1006, 0x2000);
  segmented_esc.memory[0x1008] = 0x00;
  for (uint8_t index = 0; index < 7; ++index)
  {
    segmented_esc.memory[0x1009 + index] = static_cast<uint8_t>(index + 1U);
  }
  Put32(segmented_esc.memory, 0x0220, 1U << 8U);
  segmented_core.HandleInterrupt(EscEvent::MAILBOX);
  assert(segmented_esc.memory[0x1048] == 0x20);

  Put16(segmented_esc.memory, 0x1000, 10);
  segmented_esc.memory[0x1005] = 0x03;
  Put16(segmented_esc.memory, 0x1006, 0x2000);
  segmented_esc.memory[0x1008] = 0x1D;
  segmented_esc.memory[0x1009] = 8;
  Put32(segmented_esc.memory, 0x0220, 1U << 8U);
  segmented_core.HandleInterrupt(EscEvent::MAILBOX);
  assert(segmented_esc.memory[0x1048] == 0x30);
  for (uint8_t index = 0; index < segmented_application.parameter.size(); ++index)
  {
    assert(segmented_application.parameter[index] == index + 1U);
  }
  return 0;
}
