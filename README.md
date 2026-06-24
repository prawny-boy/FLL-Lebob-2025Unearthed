# FLL Unearthed — Team LEBOB

Our robot code for the FIRST LEGO League UNEARTHED season. It runs on a LEGO
SPIKE Prime hub flashed with PyBricks, and keeps all of our competition missions
in one file with an on-hub menu for picking which one to run.

## What's here

`src/main.py` is the whole robot program: the drive base, the attachment motors,
the seven missions, and the menu you scroll through on the hub before a match.
It's deliberately one file so it's quick to download to the hub and easy to read
at the table.

## Running it

You need a SPIKE Prime hub (a Robot Inventor hub works too) running PyBricks. To
flash the firmware, open https://code.pybricks.com, click *Install Firmware*, and
follow the steps.

Then either load and run it from the PyBricks IDE (open `src/main.py`, connect to
the hub, hit *Download and Run*), or push it over Bluetooth:

```bash
pybricksdev run ble src/main.py
```

If the hub has a name, add `--name {Name}`.

## Using the menu

On start, the hub light turns blue and the display shows a mission number.

- Left and Right buttons scroll through the missions.
- Center button runs the one shown. The light goes green while it runs, then back
  to blue when it finishes.

The hub also prints the battery level to the console when it boots, so we can
check it's charged before a round.

## The robot

- Drive base: two motors, left on port D and right on port C, 62.4 mm wheels,
  130 mm axle track.
- Attachment arms: left on port F, right on port E.
- Heading is kept straight with the hub's built-in gyro.

Missions are plain functions tagged with the `@mission` decorator, which adds
them to the menu in the order they appear in the file. Alongside the normal
PyBricks drive commands there are a few "until stalled" helpers
(`straight_until_stalled`, `arc_until_stalled`, `turn_until_stalled`) for driving
into a wall or model and stopping once the robot can't move any further.

## Missions

In menu order (see `src/main.py` for the actual movements):

1. Flip boulders and heavy
2. Silo
3. Scales and raise pan
4. Ship
5. Brush and broom
6. Minecart
7. Forum, statue and flags

## Field resources

`resources/` holds the field map (`Map.pdf`) and our path sketches (`PathV1.png`
through `PathV5.png`).

## Working on it

- Keep `src/main.py` a single file with minimal imports so it stays hub-friendly.
- Follow PEP 8 (4-space indents, snake_case).
- Test on the table before committing, and note the battery voltage while you do.
- Update the path images in `resources/` if a route changes.

## License

Apache License 2.0, see [LICENSE](LICENSE). Copyright © 2026 Team LEBOB.
