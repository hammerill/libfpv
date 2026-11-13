> **Edit AGENTS.md itself.** `CLAUDE.md` (and any other agent-instruction
> file) is a symlink to it. Writing through a symlink can replace it with a
> regular file, so never edit the symlinks.

# Rules for agents working on libfpv

## Git

- Never commit or push unless explicitly told to. Leave changes in the
  working tree and say what changed.

## The kamikaze feature is dev-gatekept

Everything about kamikaze builds (`LIBFPV_KAMIKAZE`: the warhead, the bomb
arm and explode switches, the Kamikaze overlay tab, `fpv_kamikaze_*`,
`--kamikaze-demo`) stays hidden from the public:

- It may appear only inside libfpv's own code behind `LIBFPV_KAMIKAZE`
  (`#if defined(LIBFPV_KAMIKAZE)`, the CMake option), and deep in the docs
  (`docs/INTEGRATION.md` "Kamikaze builds", `docs/DECISIONS.md`).
- Never mention it in the README, install or build guides, docs/WEB.md,
  docs/PUBLISHING.md, release notes or anything else a user reads first.
- Never turn it on in CI/CD, release or deployed (web) builds.
- Game integrations must not enable it or mention it in their READMEs, mod
  pages or default builds either. Use it only in local development builds.

### Kamikaze dev machines (`LIBFPV_KAMIKAZE_DEV`)

If the environment variable `LIBFPV_KAMIKAZE_DEV` is set to `1` or `true`
(check with `echo $LIBFPV_KAMIKAZE_DEV`), this machine is a kamikaze dev
machine: every local build and deploy of libfpv, its examples and the game
integrations that use it (the `*-fpv` projects' local deploy scripts, local
CMake/Make builds) is a kamikaze build (`LIBFPV_KAMIKAZE=ON`), unless the
user says otherwise for that build.

This covers local builds only. It never changes anything public: CI/CD,
release, web or other pipelines, presets, defaults, READMEs and the rules
above all stay as they are. If the variable is unset or anything else,
build without kamikaze as usual.

### `LIBFPV_KAMIKAZE_PROMOTE`

The promote build (the Kamikaze tab pointing to the Kamikaze version on
Patreon, no warhead code) has its own rule:

- The CMake option stays `OFF` by default, in libfpv and in every game
  integration.
- Never turn it on in CI/CD, release or web pipelines yourself: no
  workflow, preset, script or default may set it. Turning it on in a
  pipeline is a manual action for the maintainer to take.
