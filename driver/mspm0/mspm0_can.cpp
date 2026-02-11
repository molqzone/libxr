#include "mspm0_can.hpp"

#if defined(__MSPM0_HAS_MCAN__)

#include <cstring>

using namespace LibXR;

namespace
{

constexpr uint32_t MSPM0_CAN_MSP_LINE_MASK =
    DL_MCAN_MSP_INTERRUPT_LINE0 | DL_MCAN_MSP_INTERRUPT_LINE1;

constexpr uint32_t MSPM0_CAN_INTR_MASK =
    DL_MCAN_INTR_SRC_RX_FIFO0_NEW_MSG | DL_MCAN_INTR_SRC_RX_FIFO1_NEW_MSG |
    DL_MCAN_INTR_SRC_TRANS_COMPLETE | DL_MCAN_INTR_SRC_TRANS_CANCEL_FINISH |
    DL_MCAN_INTR_SRC_TX_FIFO_EMPTY | DL_MCAN_INTR_SRC_BUS_OFF_STATUS |
    DL_MCAN_INTR_SRC_PROTOCOL_ERR_ARB | DL_MCAN_INTR_SRC_PROTOCOL_ERR_DATA |
    DL_MCAN_INTR_SRC_MSG_RAM_ACCESS_FAILURE;

constexpr uint32_t mspm0_can_dlc_to_len(uint32_t dlc)
{
  constexpr uint8_t LENGTH_TABLE[16] = {0, 1, 2, 3, 4, 5, 6, 7,
                                        8, 12, 16, 20, 24, 32, 48, 64};
  return LENGTH_TABLE[(dlc < 16U) ? dlc : 15U];
}

void mspm0_can_pack_to_tx_elem(const CAN::ClassicPack& pack, DL_MCAN_TxBufElement& elem)
{
  memset(&elem, 0, sizeof(elem));

  switch (pack.type)
  {
    case CAN::Type::STANDARD:
      elem.id = (pack.id & 0x7FFU) << 18U;
      elem.xtd = 0U;
      elem.rtr = 0U;
      break;

    case CAN::Type::EXTENDED:
      elem.id = pack.id & 0x1FFFFFFFU;
      elem.xtd = 1U;
      elem.rtr = 0U;
      break;

    case CAN::Type::REMOTE_STANDARD:
      elem.id = (pack.id & 0x7FFU) << 18U;
      elem.xtd = 0U;
      elem.rtr = 1U;
      break;

    case CAN::Type::REMOTE_EXTENDED:
      elem.id = pack.id & 0x1FFFFFFFU;
      elem.xtd = 1U;
      elem.rtr = 1U;
      break;

    default:
      ASSERT(false);
      break;
  }

  elem.dlc = 8U;
  elem.brs = 0U;
  elem.fdf = 0U;
  elem.efc = 0U;
  elem.mm = 0U;
  memcpy(elem.data, pack.data, sizeof(pack.data));
}

}  // namespace

MSPM0CAN* MSPM0CAN::instance_map_[MAX_CAN_INSTANCES] = {nullptr};

MSPM0CAN::MSPM0CAN(Resources res, uint32_t tx_pool_size)
    : CAN(), res_(res), tx_pool_(tx_pool_size)
{
  ASSERT(res_.instance != nullptr);
  ASSERT(res_.index < MAX_CAN_INSTANCES);
  ASSERT(instance_map_[res_.index] == nullptr);

  instance_map_[res_.index] = this;

  NVIC_ClearPendingIRQ(res_.irqn);
  NVIC_EnableIRQ(res_.irqn);

  const ErrorCode INIT_ANS = Init();
  ASSERT(INIT_ANS == ErrorCode::OK);
}

ErrorCode MSPM0CAN::Init()
{
  uint32_t timeout = INIT_TIMEOUT;
  while (!DL_MCAN_isMemInitDone(res_.instance))
  {
    if (timeout-- == 0U)
    {
      return ErrorCode::BUSY;
    }
  }

  timeout = INIT_TIMEOUT;
  while (DL_MCAN_getOpMode(res_.instance) != DL_MCAN_OPERATION_MODE_NORMAL)
  {
    if (timeout-- == 0U)
    {
      return ErrorCode::BUSY;
    }
  }

  DL_MCAN_enableIntr(res_.instance, MSPM0_CAN_INTR_MASK, true);
  DL_MCAN_enableIntrLine(res_.instance, DL_MCAN_INTR_LINE_NUM_0, true);
  DL_MCAN_enableIntrLine(res_.instance, DL_MCAN_INTR_LINE_NUM_1, true);

  DL_MCAN_clearIntrStatus(res_.instance, DL_MCAN_INTR_MASK_ALL,
                          DL_MCAN_INTR_SRC_MCAN_LINE_0);
  DL_MCAN_clearIntrStatus(res_.instance, DL_MCAN_INTR_MASK_ALL,
                          DL_MCAN_INTR_SRC_MCAN_LINE_1);

  DL_MCAN_clearInterruptStatus(res_.instance, MSPM0_CAN_MSP_LINE_MASK);
  DL_MCAN_enableInterrupt(res_.instance, MSPM0_CAN_MSP_LINE_MASK);

  return ErrorCode::OK;
}

bool MSPM0CAN::TrySendImmediate(const ClassicPack& pack)
{
  DL_MCAN_TxFIFOStatus tx_status = {};
  DL_MCAN_getTxFIFOQueStatus(res_.instance, &tx_status);

  if (tx_status.freeLvl == 0U)
  {
    return false;
  }

  DL_MCAN_TxBufElement tx_elem = {};
  mspm0_can_pack_to_tx_elem(pack, tx_elem);

  const uint32_t TX_INDEX = tx_status.putIdx;
  DL_MCAN_writeMsgRam(res_.instance, DL_MCAN_MEM_TYPE_FIFO, 0U, &tx_elem);

  return (DL_MCAN_TXBufAddReq(res_.instance, TX_INDEX) == 0);
}

ErrorCode MSPM0CAN::AddMessage(const ClassicPack& pack)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if (TrySendImmediate(pack))
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return ErrorCode::OK;
  }

  uint32_t slot = 0U;
  const ErrorCode PUT_ANS = tx_pool_.Put(pack, slot);

  ProcessTxInterrupt();

  if (primask == 0U)
  {
    __enable_irq();
  }

  if (PUT_ANS != ErrorCode::OK)
  {
    return ErrorCode::FULL;
  }

  return ErrorCode::OK;
}

void MSPM0CAN::ProcessTxInterrupt()
{
  DL_MCAN_TxFIFOStatus tx_status = {};

  while (true)
  {
    DL_MCAN_getTxFIFOQueStatus(res_.instance, &tx_status);
    if (tx_status.freeLvl == 0U)
    {
      return;
    }

    ClassicPack next_pack = {};
    if (tx_pool_.Get(next_pack) != ErrorCode::OK)
    {
      return;
    }

    if (!TrySendImmediate(next_pack))
    {
      uint32_t slot = 0U;
      const ErrorCode REQUEUE_ANS = tx_pool_.Put(next_pack, slot);
      ASSERT(REQUEUE_ANS == ErrorCode::OK);
      UNUSED(REQUEUE_ANS);
      return;
    }
  }
}

void MSPM0CAN::ProcessRxFIFO(uint32_t fifo_num)
{
  DL_MCAN_RxFIFOStatus fifo_status = {};
  fifo_status.num = fifo_num;

  while (true)
  {
    DL_MCAN_getRxFIFOStatus(res_.instance, &fifo_status);
    if (fifo_status.fillLvl == 0U)
    {
      return;
    }

    DL_MCAN_RxBufElement rx_elem = {};
    DL_MCAN_readMsgRam(res_.instance, DL_MCAN_MEM_TYPE_FIFO, 0U, fifo_num,
                       &rx_elem);

    const int32_t ACK_ANS = DL_MCAN_writeRxFIFOAck(res_.instance, fifo_num, fifo_status.getIdx);
    ASSERT(ACK_ANS == 0);
    UNUSED(ACK_ANS);

    ClassicPack pack = {};

    if (rx_elem.xtd != 0U)
    {
      pack.id = rx_elem.id & 0x1FFFFFFFU;
      pack.type = (rx_elem.rtr != 0U) ? Type::REMOTE_EXTENDED : Type::EXTENDED;
    }
    else
    {
      pack.id = (rx_elem.id >> 18U) & 0x7FFU;
      pack.type = (rx_elem.rtr != 0U) ? Type::REMOTE_STANDARD : Type::STANDARD;
    }

    const uint32_t payload_len = mspm0_can_dlc_to_len(rx_elem.dlc);
    const size_t copy_len = (payload_len < sizeof(pack.data)) ? payload_len : sizeof(pack.data);
    if (copy_len > 0U)
    {
      memcpy(pack.data, rx_elem.data, copy_len);
    }

    OnMessage(pack, true);
  }
}

void MSPM0CAN::HandleMcanLineInterrupt(DL_MCAN_INTR_SRC_MCAN line)
{
  const uint32_t intr_status = DL_MCAN_getIntrStatus(res_.instance);
  if (intr_status == 0U)
  {
    return;
  }

  DL_MCAN_clearIntrStatus(res_.instance, intr_status, line);

  if ((intr_status & DL_MCAN_INTR_SRC_RX_FIFO0_NEW_MSG) != 0U)
  {
    ProcessRxFIFO(DL_MCAN_RX_FIFO_NUM_0);
  }

  if ((intr_status & DL_MCAN_INTR_SRC_RX_FIFO1_NEW_MSG) != 0U)
  {
    ProcessRxFIFO(DL_MCAN_RX_FIFO_NUM_1);
  }

  if ((intr_status & (DL_MCAN_INTR_SRC_TRANS_COMPLETE |
                      DL_MCAN_INTR_SRC_TRANS_CANCEL_FINISH |
                      DL_MCAN_INTR_SRC_TX_FIFO_EMPTY)) != 0U)
  {
    ProcessTxInterrupt();
  }
}

void MSPM0CAN::HandleInterrupt()
{
  for (uint32_t round = 0; round < 32U; ++round)
  {
    const uint32_t msp_status =
        DL_MCAN_getRawInterruptStatus(res_.instance, MSPM0_CAN_MSP_LINE_MASK);

    if (msp_status == 0U)
    {
      return;
    }

    if ((msp_status & DL_MCAN_MSP_INTERRUPT_LINE0) != 0U)
    {
      DL_MCAN_clearInterruptStatus(res_.instance, DL_MCAN_MSP_INTERRUPT_LINE0);
      HandleMcanLineInterrupt(DL_MCAN_INTR_SRC_MCAN_LINE_0);
    }

    if ((msp_status & DL_MCAN_MSP_INTERRUPT_LINE1) != 0U)
    {
      DL_MCAN_clearInterruptStatus(res_.instance, DL_MCAN_MSP_INTERRUPT_LINE1);
      HandleMcanLineInterrupt(DL_MCAN_INTR_SRC_MCAN_LINE_1);
    }
  }
}

void MSPM0CAN::OnInterrupt(uint8_t index)
{
  if (index >= MAX_CAN_INSTANCES)
  {
    return;
  }

  MSPM0CAN* can = instance_map_[index];
  if (can == nullptr)
  {
    return;
  }

  can->HandleInterrupt();
}

#endif
