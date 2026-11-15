# Publishing a libfpv mod

Once a mod works, players should be able to use it without building
anything, reading source code or hunting for dependencies. The target:

> You send one link. The player opens it, reads a short numbered list,
> downloads one ZIP, extracts it into the game folder and flies.

This file describes how to get there with almost no ongoing work. The
reference implementation is
[gtaiv-fpv](https://github.com/hammerill/gtaiv-fpv): see its
`.github/workflows/release.yml` and `docs/INSTALL.md`.

## The setup in one paragraph

A GitHub Actions workflow runs on every push to `main`. It checks out the
mod and the latest libfpv, builds both for the game's platform and bitness,
packs the result into a ZIP whose contents go straight into the game
folder, and publishes it as a new GitHub release. The release text is a
player's install guide kept in the mod repository. Because the ZIP always
has the same name, two links never change:

- `https://github.com/<you>/<mod>/releases/latest`: the page you send to
  players.
- `https://github.com/<you>/<mod>/releases/latest/download/<mod>.zip`: a
  direct download of the newest build.

## The ZIP

- **Mirror the game folder.** The files in the ZIP sit at the paths where
  they belong in the game, relative to the folder players extract into. No
  extra top-level folder (`mod-v3/...`), so "extract everything here" is
  the whole instruction.
- **Ship everything you built:** the mod's own files and `libfpv.dll` (or
  `.so`/`.dylib`), built for the game's bitness. A 32-bit game needs the
  i686 build (`cmake/toolchains/mingw-w64-i686.cmake`).
- **A split build** (`LIBFPV_SPLIT_PROCESS`, INTEGRATION.md "Split
  builds") is four things, all in the library's folder: `libfpv.dll`,
  `libfpv-server.exe`, and the `libfpv-fonts/` and `libfpv-presets/`
  folders. Ship all four. Without the server the drone can't start, and
  without the folders the OSD is blank and only one quad is left. List them
  in the release page's folder tree and uninstall list too.
- **Ship libfpv's license** as `libfpv.LICENSE.txt` (see
  [Licensing](#licensing)).
- **Don't bundle other people's tools** (ASI loaders, script hooks, mod
  managers). Their licenses may not allow it, they release updates without
  you, and players may already have them. Link them instead.
- **Keep the name stable** (`<mod>.zip`, no version or date in it), so the
  `latest/download` link keeps working. The release title and tag carry the
  version.
- **Settings go next to the mod** (for example `<game>/fpv-data/`), and the
  ZIP must not contain them, so extracting a new build over an old one keeps
  the player's radio setup and rates.

## The release page

Write it as `docs/INSTALL.md` in the mod repository; the workflow uses it as
the release text. Write it for someone who has never modded the game:

1. **One download link at the top** for the mod's ZIP.
2. **Numbered steps**, one per thing to install, in order:
   1. The exact game edition and version, where to buy it, and how to find
      the game folder (for example "Steam > right-click > Manage > Browse
      local files").
   2. Each prerequisite: a **direct link to the asset to download** (not
      only the project page), and which file should end up where (for
      example "`dinput8.dll` should be next to `GTAIV.exe`"). Link the project's
      releases page too, in case the asset name changes.
   3. The mod itself: "download, extract everything into the game folder".
3. **The folder tree** the player should see afterwards, with each file
   labelled by the step it came from. Most "it doesn't work" reports are a
   missing or misplaced file, and the tree lets players check themselves.
4. **How to fly**: a small controls table (for example F7: launch the
   drone, F8: settings, plus the mod's camera, respawn and other keys),
   then one line on setting up a radio (F8 > radio wizard) and on the
   keyboard fallback.
5. **Betaflight Configurator**: tell players they can tune the drone like a
   real quad. In [Configurator](https://app.betaflight.com), turn on manual
   connection mode in Options, pick Manual Selection, enter
   `ws://localhost:5761` as the port and connect, while the drone is
   running in the game. Mention that F8 > Configurator shows the same
   address, and that Betaflight won't arm while Configurator is connected.
6. **Updating** (extract the new ZIP over the old files) and
   **uninstalling** (the exact list of files and folders to delete).
7. **Troubleshooting**: the one or two most common mistakes, where the log
   file is, and where to report a problem with that log attached.

Leave out build instructions, architecture notes and history; those belong
in the README for developers. Keep the README's install section to a link to
the release page, so there is one set of player instructions to maintain.

The workflow appends a "This build" section with the mod and libfpv commits
it was built from. That tells you which libfpv a bug report is about, and it
points players to the exact source.

## The workflow

Copy this into `.github/workflows/release.yml` and adjust the build and
package steps (gtaiv-fpv's version is a working example for a 32-bit
Windows game):

```yaml
name: Release
on:
  push:
    branches: [main]
  workflow_dispatch:          # rebuild by hand, e.g. for a new libfpv
permissions:
  contents: write             # needed to create releases
concurrency:
  group: release
  cancel-in-progress: false

jobs:
  release:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4
        with: { path: mod }
      - uses: actions/checkout@v4
        with:
          repository: hammerill/libfpv
          path: libfpv
          submodules: true    # not recursive: SimITL's nested ones are SSH
      - run: |
          sudo apt-get update
          sudo apt-get install -y --no-install-recommends \
            gcc-mingw-w64-i686-posix g++-mingw-w64-i686-posix zip
      - name: Build libfpv
        run: |
          cmake -S libfpv -B libfpv/build \
            -DCMAKE_TOOLCHAIN_FILE="$GITHUB_WORKSPACE/libfpv/cmake/toolchains/mingw-w64-i686.cmake" \
            -DLIBFPV_BUILD_TESTS=OFF -DLIBFPV_BUILD_EXAMPLES=OFF
          cmake --build libfpv/build -j"$(nproc)"
      - name: Build the mod
        run: |
          cmake -S mod -B mod/build -DLIBFPV_DIR="$GITHUB_WORKSPACE/libfpv" # + toolchain
          cmake --build mod/build -j"$(nproc)"
      - name: Package
        run: |
          mkdir -p dist/pkg
          cp mod/build/<your files> libfpv/build/libfpv.dll dist/pkg/
          cp libfpv/LICENSE dist/pkg/libfpv.LICENSE.txt
          (cd dist/pkg && zip -9r ../mod.zip .)
      - name: Publish
        env: { GH_TOKEN: "${{ github.token }}" }
        run: |
          { cat mod/docs/INSTALL.md
            echo; echo "## This build"; echo
            echo "- mod $(git -C mod rev-parse --short HEAD)"
            echo "- libfpv https://github.com/hammerill/libfpv/commit/$(git -C libfpv rev-parse HEAD)"
          } > dist/notes.md
          gh release create "build-$GITHUB_RUN_NUMBER" dist/mod.zip \
            --repo "$GITHUB_REPOSITORY" --target "$GITHUB_SHA" \
            --title "Build $GITHUB_RUN_NUMBER" --notes-file dist/notes.md --latest
```

Notes:

- **Build libfpv from source in the workflow** instead of committing a DLL.
  The release then always has a known libfpv commit, and nobody has to
  remember to copy a new DLL in.
- **Turn off libfpv's tests and examples** in release builds. They are not
  shipped, and the build is much faster without them.
- **Rolling releases** (one per push, tagged `build-N`) need nothing from
  you beyond pushing. If you want hand-picked versions instead, trigger on
  `push: tags: ['v*']` and tag the commits you want to release. Keep the
  same ZIP name either way.
- **New libfpv, no mod change:** run the workflow by hand from the Actions
  tab. To do it automatically, add `repository_dispatch: { types:
  [libfpv-updated] }` to the mod's triggers, and give libfpv a workflow that
  calls `gh api repos/<you>/<mod>/dispatches -f event_type=libfpv-updated`
  with a fine-grained token (Contents: read and write on the mod repo)
  stored as a secret. A libfpv change can break a mod, so only do this when
  you trust libfpv's `main`, or pin libfpv to a tag with the checkout's
  `ref:` and bump it on purpose.

## Web releases

A game with its source available can also be published as a web page
([WEB.md](WEB.md)): then the one link *is* the game. Players open it and fly,
with nothing to download or extract and no game folder to find. The page
is static files, so GitHub Pages can host it for free from the same
repository, rebuilt on every push like the ZIP:

```yaml
name: Web
on:
  push:
    branches: [main]
  workflow_dispatch:
permissions:
  contents: read
  pages: write                # publish to GitHub Pages
  id-token: write
concurrency:
  group: pages
  cancel-in-progress: true

jobs:
  web:
    runs-on: ubuntu-24.04
    environment:
      name: github-pages
      url: ${{ steps.deploy.outputs.page_url }}
    steps:
      - uses: actions/checkout@v4
        with: { path: mod }
      - uses: actions/checkout@v4
        with:
          repository: hammerill/libfpv
          path: libfpv
          submodules: true    # not recursive: SimITL's nested ones are SSH
      - name: Emscripten
        run: |
          git clone --depth 1 https://github.com/emscripten-core/emsdk.git
          emsdk/emsdk install 6.0.12 && emsdk/emsdk activate 6.0.12
      - name: Build libfpv
        run: |
          source emsdk/emsdk_env.sh
          emcmake cmake -S libfpv -B libfpv/build-web -DLIBFPV_BUILD_TESTS=OFF -DLIBFPV_BUILD_EXAMPLES=OFF
          cmake --build libfpv/build-web -j"$(nproc)"
      - name: Build the mod
        run: |
          source emsdk/emsdk_env.sh
          emcmake cmake -S mod -B mod/build-web -DLIBFPV_DIR="$GITHUB_WORKSPACE/libfpv"  # or emmake make ...
          cmake --build mod/build-web -j"$(nproc)"
      - name: Package
        run: |
          mkdir site
          cp mod/build-web/<index.html, .js, .wasm, game data> site/
          cp libfpv/LICENSE site/libfpv.LICENSE.txt
      - uses: actions/upload-pages-artifact@v3
        with: { path: site }
      - id: deploy
        uses: actions/deploy-pages@v4
```

Turn Pages on once in the repository's Settings > Pages ("Source: GitHub
Actions"). The page is then at `https://<you>.github.io/<mod>/`.

- **Game data.** If the game needs files the player owns (a ROM, the
  original game's assets), the page can't ship them. It has to ask the
  player for them (a file picker that writes them into the module's file
  system) before the game starts. Ship only what you may distribute.
- **What players need to know** fits on the page itself: the controls, "move
  a stick on your radio so the browser sees it", and that settings are
  saved in this browser (WEB.md, "Radios and gamepads" and "Limits").
- **Licensing** is the same as for the ZIP (below): put the license on the
  site, and link the source repository and the exact libfpv commit from the
  page.

## Licensing

libfpv is GPL-3.0-or-later because it embeds Betaflight. Shipping
`libfpv.dll` means:

- the mod must be under GPL-3.0-compatible terms (state its license in the
  repository);
- the ZIP includes the license text (`libfpv.LICENSE.txt`);
- players can get the corresponding source. The "This build" section links
  the exact libfpv commit, and GitHub attaches the mod's source to every
  release.

A split build keeps the GPL parts out of the library the game loads:
Betaflight and SimITL run in `libfpv-server`, a separate program, and the
fonts and shipped presets are separate files. The library itself (libfpv's
client side) is under MIT as well as the GPL (`LICENSE-MIT`), so the mod's
game-side code can be under other terms. Whether a separate process is
enough separation for your mod is your call to make; the split exists so
that the option is there. The GPL parts still ship, so:

- `libfpv-server`, `libfpv-fonts/` and `libfpv-presets/` stay GPL-3.0. The
  folders carry the license text, and the ZIP includes it next to the
  server as well (`libfpv.LICENSE.txt`);
- players can get their source: link the exact libfpv commit, as the
  "This build" section does;
- ship `LICENSE-MIT` (as `libfpv.LICENSE-MIT.txt`) for the library, since
  the MIT terms ask for the notice to go with it.

## Before you send the link

- Install from the release page into a clean copy of the game, following
  only what the page says. Anything you had to do that the page does not
  say is a missing step.
- Check that every link on the page works, including the
  `latest/download` ones (they only work once a release exists).
- Extract a new build over an old one and check that settings survive.
- Check the uninstall list against what the mod actually creates (logs,
  data folders, copies made at runtime).
