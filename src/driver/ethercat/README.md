# XRECAT

XRECAT is an EtherCAT slave driver subtree for LibXR. It follows the same standalone-repository
and LibXR-subtree layout as XRUSB.

## Layout

- `core/`: LibXR-facing EtherCAT API.
- `device/slave/`: SOES slave lifecycle and profile selection.
- `device/slave/profile/`: object dictionaries and application-specific PDO profiles.
- `port/hpm/`: HPM ESC/PDI implementation.

## CMake

Enable the subtree through LibXR and supply a SOES checkout:

```cmake
set(LIBXR_XRECAT_ENABLE ON)
set(LIBXR_ETHERCAT_SOES_DIR "/path/to/SOES/soes")
set(LIBXR_ETHERCAT_PROFILE rm) # rm or foc
```

`rm` is the default profile. Each build selects exactly one profile and its object dictionary.
