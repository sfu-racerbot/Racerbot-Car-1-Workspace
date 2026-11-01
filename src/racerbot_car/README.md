# Car identity and profiles

> **Who this is for:** developers configuring the workspace (the shared project folder) for a car or simulator.
> **Read first:** [Hardware reference](../../docs/hardware-reference.md) for the car's devices.
> **What's in it:** identity resolution, profile locations and adding a car.
> **Time:** about 5 minutes.

`racerbot_car` identifies the car, loads its hardware profile and locates its data.

## Highlights

- Resolves a car ID from the Jetson serial or a development override.
- Refuses a conflicting override rather than selecting the wrong car.
- Validates hardware profiles and locates per-car data without creating folders.

## Identity resolution

1. A Jetson serial listed in `cars/machines.yaml` selects its car. A non-empty
   `RACERBOT_CAR` that disagrees causes an error.
2. For an unlisted serial, a known `RACERBOT_CAR` selects that car.
3. An unknown `RACERBOT_CAR` causes an error.
4. Without either a known serial or an override, resolution fails with instructions.

## Files and adding a car

`cars/machines.yaml` maps quoted Jetson serials to car IDs. Each `cars/rbN/car.yaml`
contains the hardware profile. Data paths are `data/rbN/KIND` in the workspace;
`RACERBOT_WS` overrides the default `~/racerbot-ws`. `RACERBOT_CARS_DIR` overrides
the profile directory.

To add a car, create an `rbN` folder with a `car.yaml` matching the existing
profile schema. Add its quoted Jetson serial and ID to `machines.yaml` alongside
the folder. Check each hardware value against the car before using the profile.

## Overlays

An overlay is a ROS parameter YAML file that supplies settings for one car on
top of a shared configuration. For a car ID and config filename, the files are
loaded in this order:

1. The shared config.
2. `cars/ID/calibration/FILENAME` for hardware calibration.
3. `cars/ID/tuning/FILENAME` for deliberate per-car tuning.

Only existing overlays are loaded. Use the same node block key as the shared
file for each overridden parameter: ROS keeps differently spelled block keys
as separate entries, so a later file does not always win. A node is a running
ROS program; see the [glossary](../../docs/glossary.md).

The calibration allow-list and required keys are in
[`guards.py`](racerbot_car/guards.py). If a required calibration key is missing,
`layered()` refuses to continue; otherwise the node might silently use its
declared default for that hardware.

## Tests

From a terminal at the workspace root, run:

```bash
python3 -m pytest src/racerbot_car/test -q
```

Pytest reports the number of passing tests; failures include the offending assertion.
