# Shipped presets

The drones (`quad/`, each a `.json` with an optional Betaflight
`.diff.txt`) and the components they are built from (`motor/`,
`propeller/`, `battery/`, `vtx/`, `camera/`) that libfpv writes into the
player's preset folder on first launch and after an update. Some of them
are exports of SimITL's own presets (https://github.com/AJ92/SimITL,
GPL-3.0); like the rest of libfpv's assets they are GPL-3.0.

libfpv compiles them into the library. Split builds
(`LIBFPV_SPLIT_PROCESS`) don't: they ship in a `libfpv-presets` folder next
to the library, with this file and the GPL-3.0 text, and are read from
there (`LIBFPV_SHIPPED_PRESET_DIR` points elsewhere).
