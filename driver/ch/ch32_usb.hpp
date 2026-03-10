#pragma once

#include "libxr_def.hpp"
#include DEF2STR(LIBXR_CH32_CONFIG_FILE)

#if !defined(LIBXR_CH32_HAS_USB_DEV_FS) && !defined(USBFSD) && \
    (defined(RCC_APB1Periph_USB) || defined(RCC_AHBPeriph_USBFS))
#define LIBXR_CH32_HAS_USB_DEV_FS 1
#endif

namespace LibXR
{

/**
 * @brief CH32 USB 控制器编号 / CH32 USB controller identifiers
 */
typedef enum : uint8_t
{
#if defined(USBFSD)
  CH32_USB_OTG_FS,
#endif
#if defined(USBHSD)
  CH32_USB_OTG_HS,
#endif
  CH32_USB_DEV_ID_NUM
} ch32_usb_dev_id_t;

}  // namespace LibXR
