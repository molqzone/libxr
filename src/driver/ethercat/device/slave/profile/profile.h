#ifndef LIBXR_ETHERCAT_PROFILE_H
#define LIBXR_ETHERCAT_PROFILE_H

#ifdef __cplusplus
extern "C" {
#endif

void ethercat_profile_init(void);
void ethercat_profile_on_cycle(void);
void ethercat_profile_get_inputs(void);
void ethercat_profile_set_outputs(void);
void ethercat_profile_safe_outputs(void);

#ifdef __cplusplus
}
#endif

#endif /* LIBXR_ETHERCAT_PROFILE_H */
