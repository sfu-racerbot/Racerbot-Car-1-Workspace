# Car identity and profiles

> **Who this is for:** developers configuring the workspace for a car or simulator.
> **Read first:** [Hardware reference](../../docs/hardware-reference.md) for the car's devices.
> **What's in it:** identity resolution, profile locations and adding a car.

`racerbot_car` identifies the car, loads its hardware profile and locates its data.

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

## Tests

From a terminal at the workspace root, run:

```bash
python3 -m pytest src/racerbot_car/test -q
```

Pytest reports the number of passing tests; failures include the offending assertion.
