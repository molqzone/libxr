#pragma once

#include "sdi_general_gpio.hpp"

namespace LibXR::Debug
{
/**
 * @brief Backward-compatible alias.
 *
 * This keeps previous call sites building while the implementation now lives
 * in `SdiGeneralGPIO`.
 */
template <typename BitPort>
using SdiOverSwd = SdiGeneralGPIO<BitPort>;

}  // namespace LibXR::Debug

