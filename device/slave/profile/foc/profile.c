#include "profile.h"

#include <string.h>

#include "utypes.h"

_Objects Obj = {
    .serial = 1,
};

void ethercat_profile_init(void) {}

void ethercat_profile_on_cycle(void)
{
    (void)Obj.controlword;
}

void ethercat_profile_get_inputs(void) {}

void ethercat_profile_set_outputs(void) {}

void ethercat_profile_safe_outputs(void)
{
    Obj.controlword = 0;
    Obj.desired_joint_torque = 0.0F;
    Obj.desired_joint_position = 0.0F;
    Obj.desired_joint_velocity = 0.0F;
}
