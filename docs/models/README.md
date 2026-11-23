# Drone models for game integrations

Ready-made drone models in engine-neutral formats, for integrations to turn
into the game's own model format, so the game renders the drone itself
(shadows, light, fog, reflections): see
[INTEGRATION.md, "The drone model"](../INTEGRATION.md#the-drone-model).

| Model | What it is | Files |
|---|---|---|
| `quad5` | A 5" freestyle quad, libfpv's `freestyler-5` preset: true-X frame, 2207 motors, 5.1" props, a 6S battery on top, a camera tilted up 25°. 1404 triangles. First made for Portal 2. | [quad5/](quad5/) |

Each model folder holds:

- `<name>.gltf`: glTF 2.0 with its buffer embedded (a single file). Blender,
  Sollumz, Unity, Godot and most converters import it.
- `<name>.obj` + `<name>.mtl`: the same geometry for tools without glTF.
- `<name>.json`: what a converter needs to know: units, axes, the parts,
  each motor by libfpv's motor index, the camera, the material colours.

`make_models.py` generates all of them (standard library only):

```sh
python3 -I docs/models/make_models.py          # every model
python3 -I docs/models/make_models.py quad5    # one
```

Add a model by writing a function that builds its parts and registering it
in `MODELS`. Keep the conventions below, and extend `check()` so a turned or
mirrored model fails the build.

## Conventions

- **Units:** metres. Scale for the engine (Source: 39.37 units per metre).
- **Origin:** the drone's centre, which is `frame.position`. The motor hubs
  are about level with it.
- **Axes in the .gltf and .obj:** glTF's own convention: **+Z forward (the
  nose, the camera), +Y up, +X left** (right-handed). Positions in the
  `.json` name their axes (`forward`, `left`, `up`) instead.
- **Parts** (one glTF node and mesh each, one OBJ object each):
  - `body`: the frame, stack, battery, camera, antenna and motors. Always
    shown.
  - `prop_still_<i>`: motor *i*'s prop standing still.
  - `prop_spin_<i>`: motor *i*'s prop spinning, as a translucent disc.
  - Per motor, show `prop_spin_<i>` above about 1500 rpm
    (`frame.motors[i].rpm`), otherwise `prop_still_<i>`. Show neither once
    `frame.motors[i].prop_damage >= 1`.
- **Motor index** *i* is libfpv's (`frame.motors[i]`), which is Betaflight's
  motor *i*+1 in Quad X: 0 rear right, 1 front right, 2 rear left, 3 front
  left.
- **Materials:** flat colours (`baseColorFactor` in the glTF, `Kd` in the
  MTL, sRGB values in the JSON). `propblur` is translucent (alpha 70/255).
  No textures, so a converter only needs one small texture per colour if
  the engine requires textures.

## Converting for a game

The importer's axis settings are where models go wrong
([INTEGRATION.md](../INTEGRATION.md#the-drone-model): Portal 2's model came
out a quarter turn off). After converting, check in game, in the chase view:
the nose points where the drone flies, a roll to the right lowers the props
on the right, and each prop sits where `frame.motors[i].position` says.

- **Blender** (the usual hub: Sollumz for RAGE games, and the exporters for
  Unreal, Unity and FBX): File > Import > glTF 2.0. Blender turns glTF's
  +Y up into its +Z up. The nose then points along Blender's **-Y**, with
  the left side along +X.
- **Source (Portal 2):** portal-2-fpv's `model/build_model.py` writes the
  same geometry as SMD (in inches, X forward, Y left, Z up), with a QC that
  puts the props in a body group, and compiles it with studiomdl. The
  compiled model's nose ended up along +Y, so the mod yaws the entity 90°
  right.
- **RAGE (GTA V, RDR2):** from Blender with Sollumz (Sollumz_RDR for RDR2),
  export a drawable as CodeWalker/CodeX XML, then build the binary `.ydr`
  (CodeWalker for GTA V, CodeX for RDR2) and an archetype for it in a
  `.ytyp`. Then stream both: FiveM/RedM resources, or a mod loader's stream
  folder in single player (RDR2: Lenny's Mod Loader).
- **Fixed-function or hand-drawn renderers** (GTA SA style): read the OBJ
  or the glTF at build time, and bake it into an array in the mod.
