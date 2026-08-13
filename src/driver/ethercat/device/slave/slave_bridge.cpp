#include "slave.hpp"

extern "C" void cb_get_inputs(void)
{
    LibXR::EtherCAT::Slave::DispatchInputs(ethercat_slave_in_isr());
    ethercat_profile_get_inputs();
}

extern "C" void cb_set_outputs(void)
{
    ethercat_profile_set_outputs();
    LibXR::EtherCAT::Slave::DispatchOutputs(ethercat_slave_in_isr());
}

extern "C" void ethercat_safe_outputs(void)
{
    ethercat_profile_safe_outputs();
    LibXR::EtherCAT::Slave::DispatchSafeOutputs(ethercat_slave_in_isr());
}
