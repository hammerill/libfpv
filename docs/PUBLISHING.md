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

## Mod repositories: Nexus Mods, Steam Workshop, Thunderstore, mod.io

A mod can also be uploaded to the game's mod repositories, so players who
browse there find it. Keep the page short and send players to the mod's
libfpv.com page for everything that changes (the install guide, the
requirements, new builds), so there is still one set of instructions to
maintain. Use the same names and wording on every repository, so libfpv
mods read as one family.

**Title.** `FPV Drone via libfpv`. When the repository needs the game's
name in the title (a site that lists mods of all games together, or a
moderator who asks for it), use `FPV Drone via libfpv for <game>` with the
game's full name: `FPV Drone via libfpv for Red Dead Redemption 2`. Leave
out "via libfpv" only when the user asks for it, for example when the game
is new and this is its first FPV mod: `FPV Drone for GTA 6`. Where the name
can only have letters, digits and underscores (Thunderstore), use
`FPV_Drone_via_libfpv`; the site shows the underscores as spaces. Inside
the game the mod is still `libfpv` (INTEGRATION.md, "Label the mod
`libfpv` inside the game") and F8 > About still says `libfpv for <game>`.

**Summary** (the one-line description lists show; keep it under 250
characters, the limit of Thunderstore and mod.io). Start with libfpv.com's
headline, then name the game and what the player can do:

> Real Betaflight firmware flying across the games you know well. libfpv
> puts an FPV drone into Red Dead Redemption 2: fly it with your own radio,
> gamepad or keyboard, and tune it in Betaflight Configurator like a real
> quad.

List only the ways of flying the mod supports.

**Description** (the page body), in this order:

1. The summary.
2. The game's blurb from its libfpv.com page ("Arthur puts a quad down on
   the ground in front of him. Chase wagons across the plains...").
3. A short feature list: libfpv's features first (real Betaflight
   firmware, your own radio, a gamepad or the keyboard, Betaflight
   Configurator at `ws://localhost:5761`, the OSD, rates, presets and
   cameras in the F8 overlay), then one or two of the game's own (what the
   mod hooks: traffic, people, water, police).
4. One line to the mod's page: "Install guide, requirements and updates:
   https://libfpv.com/games/<id>". Don't copy the install steps, the
   folder tree or the controls table into the repository; they go stale.
5. Credits and licences: Betaflight and SimITL, libfpv (GPL-3.0-or-later,
   its client library also MIT), the mod's own licence and the
   game-side libraries it uses, and a link to the source ("Licensing"
   below).

**Media.** First the user's own footage of the drone flying in the game:
several screenshots, or a single video (a GIF counts too). Ask the user
for it; don't make it from something else or generate it. Last, the
libfpv logo, `docs/img/libfpv-logo.png` in this repository
(https://raw.githubusercontent.com/hammerill/libfpv/main/docs/img/libfpv-logo.png).
Where the repository picks a thumbnail from the first image, that keeps
the game in it.

**Category and tags.** The repository's own category for gameplay or
vehicles (whichever fits a drone you fly), and the tags `FPV`, `drone`,
`quadcopter`, `Betaflight` and `libfpv`, as far as the site allows. Add any
tag the site requires (game version, single player).

**Version and files.** The version is the release's build number (`Build
42`, or `42` where only numbers are allowed). Upload the same ZIP as the
mod's release, under the same name, and describe the file as "Build 42,
2026-11-27". Upload each new build there as well, or don't upload files at
all and send players to the mod's page instead: an old file left on the
repository becomes the build players get.

**Requirements and links.** Put each prerequisite (an ASI loader, a script
hook, a recompilation) in the repository's requirements field, with a
link, so its tools warn players who miss one. Put the mod's libfpv.com
page in the links or homepage field. Bug reports go wherever that page
says, with the log and F8 > About > "Copy build info" attached.

**Author.** The user's own account uploads the mod and is its author; the
credits name libfpv and Betaflight. Where a repository asks about
AI-generated content (art, sound, code), answer it truthfully. For a
libfpv integration the level is "AI-assisted", and when the form asks
"Describe how AI assisted you", the answer is:

> The core libfpv integration is AI-driven. The libfpv integration spec is
> defined and constantly updated by a human; as well as testing, polishing
> and debugging.

Mention any AI-generated art or sound in the mod on top of that.

**Credits (the short field).** Where a repository has a short credits
field (as opposed to the credits in the description), it reads:

> Thanks to devs of SimITL and pr0p for their amazing FPV physics engine I
> can now embed in any game.

### "Prepare for publishing on mod repos"

When the user asks for this (or anything like it: "get the Nexus page
ready", "what do I put on Thunderstore"), don't upload anything and don't
log in anywhere. Gather everything the user will paste into the upload
forms, so filling them in is copy and paste:

1. **Collect the facts from the integration**, not from memory: the
   game's full name, the libfpv.com page (test it as INTEGRATION.md "The
   About tab" says, and if it fails ask the user at the end), the latest
   release's build number, date and ZIP name, the prerequisites from
   `docs/INSTALL.md`, the ways of flying and the game's own hooks from the
   README and the code, the licences from "Licensing", and the game's
   blurb from its libfpv.com page.
2. **Fill in every field** from the conventions above, adapted to the
   game, and check each against the limits you know (250 characters for
   the summary on Thunderstore and mod.io, letters, digits and underscores
   for a Thunderstore name). Say which limits you could not confirm.
3. **Put the media in a new temporary folder** (the agent's scratchpad,
   or `mktemp -d`), numbered in upload order: the user's screenshots or
   video first (`01-flying.png`, `02-...`), the libfpv logo last
   (`09-libfpv-logo.png`, copied from `docs/img/libfpv-logo.png`). Copy
   the files; don't move, resize or convert the user's originals. If the
   user hasn't given any footage, do everything else and ask for it at the
   end; don't make or generate any.
4. **Reply with:**
   - a table of the short fields, one row per field (title, title with
     the game's name, Thunderstore name, version, file name, file
     description, category, tags, requirements, links, author, AI level),
     with a column per repository where they differ;
   - every field that is a sentence or longer, or may need more than one
     line, outside the table, each in its own code block under its field's
     name: the summary (with its length), the short credits, the AI
     answer above, and
     anything else of that kind. Text in a table cell wraps across cells
     and can't be copied cleanly from a terminal (Claude Code's CLI, for
     one), while a code block copies as one piece;
   - the description in its own code blocks, because a table can't hold
     it: one in BBCode (Nexus Mods, Steam Workshop) and one in Markdown
     (mod.io, Thunderstore's README);
   - the media folder's full absolute path, and its Windows path too when
     the agent runs in WSL (`wslpath -w <folder>`), with the files in
     upload order;
   - what is still missing (footage, an unconfirmed URL or limit), asked
     as questions at the end.

Write nothing of this into the repository: the values change with every
build, and the next request gathers them again.

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
