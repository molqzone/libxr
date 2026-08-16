#pragma once

#include <cstdint>

namespace LibXR::EtherCAT::EscRegister
{

inline constexpr uint16_t AL_CONTROL = 0x0120U;
inline constexpr uint16_t AL_STATUS = 0x0130U;
inline constexpr uint16_t AL_STATUS_CODE = 0x0134U;
inline constexpr uint16_t AL_EVENT_REQUEST = 0x0220U;
inline constexpr uint16_t FMMU_COUNT = 0x0004U;
inline constexpr uint16_t SYNC_MANAGER_COUNT = 0x0005U;
inline constexpr uint16_t FMMU_BASE = 0x0600U;
inline constexpr uint16_t SYNC_MANAGER_BASE = 0x0800U;

inline constexpr uint16_t SYNC_MANAGER_SIZE = 8U;
inline constexpr uint16_t FMMU_SIZE = 16U;
inline constexpr uint8_t MAX_SYNC_MANAGER_COUNT = 8U;
inline constexpr uint8_t MAX_FMMU_COUNT = 16U;

inline constexpr uint32_t EVENT_AL_CONTROL = 1U << 0U;
inline constexpr uint32_t EVENT_SYNC_MANAGER_CHANGE = 1U << 4U;
constexpr uint32_t SyncManagerEvent(uint8_t index) { return index < 16U ? (1U << (8U + index)) : 0U; }

inline constexpr uint8_t SYNC_MANAGER_ENABLE = 0x01U;
inline constexpr uint8_t SYNC_MANAGER_OPERATION_MODE_MASK = 0x03U;
inline constexpr uint8_t SYNC_MANAGER_BUFFERED_MODE = 0x00U;
inline constexpr uint8_t SYNC_MANAGER_MAILBOX_MODE = 0x02U;
inline constexpr uint8_t SYNC_MANAGER_DIRECTION_MASK = 0x0CU;
inline constexpr uint8_t SYNC_MANAGER_ECAT_READ = 0x00U;
inline constexpr uint8_t SYNC_MANAGER_ECAT_WRITE = 0x04U;
inline constexpr uint8_t SYNC_MANAGER_STATUS_MAILBOX = 0x08U;

inline constexpr uint8_t FMMU_READ_ENABLE = 0x01U;
inline constexpr uint8_t FMMU_WRITE_ENABLE = 0x02U;
inline constexpr uint8_t FMMU_ENABLE = 0x01U;

}  // namespace LibXR::EtherCAT::EscRegister
