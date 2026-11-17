# OSD fonts

`betaflight-default.mcm` and `betaflight-bold.mcm` are copied unmodified from
Betaflight Configurator (`resources/osd/2/default.mcm` and `bold.mcm`,
commit `3afbb0e`, GPL-3.0): https://github.com/betaflight/betaflight-configurator

MAX7456 `.mcm` format: a `MAX7456` header line, then 256 glyphs × 64 lines.
Each line is one byte written as 8 binary digits. Each glyph is 12×18
pixels at 2 bits per pixel: `00` black, `10` white, `01`/`11`
transparent. Only the first 54 bytes of each glyph are pixels; the last
10 are padding. libfpv embeds the files and turns them into an RGBA atlas
at runtime (`src/client/osd/osd.cpp`). Split builds (`LIBFPV_SPLIT_PROCESS`)
don't embed them: they ship in a `libfpv-fonts` folder next to the library,
with this file and the GPL-3.0 text, and are read from there.
