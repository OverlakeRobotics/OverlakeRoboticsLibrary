# Overlake Robotics Library

The Overlake Robotics Library is a movement library meant for FTC teams to control their robot
during both the autonomous and TeleOp period. It works best with an odometry system, especially
GoBilda Pinpoint, but also works with just encoders.

This library is designed to be easy to use, with a custom path planner interface you can use to
design your autonomous with zero code. Autonomous paths run on Pedro Pathing, which takes one
tuning session on your robot (see below).

## Requirements
- Holonomic drivetrain (X-Drive or Mecanum)
- Motor encoders (even if you have odometry)
- Optional but recommended: Odometry system

## Quick Start
1. Fork this repository and clone that fork to your computer.
2. Upload the code to the robot (Guide to do this with ADB: https://ftc-tech-toolbox.vercel.app/docs/Getting%20Started/wd).
3. Run some of the example OpModes and look through them. (Make sure to remove the @Disabled in the code for the ones you want to run)
4. Optional: Run the tuning programs to get better autonomous movement (Requires an odometry system)
5. Make your own OpModes using the movement library!

## Tuning Pedro Pathing
Do this once per robot, before running paths. The planner gets its timing from these numbers.

1. Connect to the robot and open AutoTune at http://192.168.43.1:10158.
2. Run the **Mecanum** tuner, paste the motor directions into `pedro/Constants.java`, and redeploy.
3. Run the **Pinpoint** tuner, put the pod offsets in `X_POD_OFFSET_INCHES` / `Y_POD_OFFSET_INCHES`,
   update the pod directions, and redeploy.
4. Run the **Foresight** tuner, paste the config over `foresightConfig`, set `PEDRO_TUNED = true`,
   and redeploy.
5. Init your path OpMode and connect the planner. The Playback card should now say "Using robot
   limits" with your robot's top speed. If it says the robot is not tuned, check step 4.

Run the tuners again after changing the drivetrain or weight, then repeat from the tuner that
changed. The planner picks up new numbers by itself the next time it connects or uploads. It uses
your speeds, but the tuners do not measure acceleration, so set that in the planner under Setup.

No Pinpoint or Mecanum? Change `create()` in `Constants`. `PathPlanLegacy` is the old point-chasing
follower: it needs no Pedro tuning apart from the pod offsets, but it does not run your tag list.

## Path Planner
To make full use of this movement library, use the Overlake Robotics Path Planner:
https://github.com/OverlakeRobotics/OverlakeRoboticsPathPlanner

You can upload paths instantly to the robot, with custom actions like intaking at any
point. Once your actions are in your OpMode, you can build a whole autonomous with zero code.

Extend `PathPlanAuto` (see `PathPlanExample`). You get your hardware in `setup()` and list your
tags in `tags()`, one entry per tag:

```java
@Override
protected void tags() {
    tag("intake", power -> intake.setPower(power));   // runs once, gets the tag's value
    tag("myTag", () -> mechanism.go());               // runs once

    stopTag("myStopTag")                              // stops the robot until it is finished
            .start(() -> mechanism.begin())
            .loop(() -> mechanism.update())
            .until(() -> mechanism.isDone())          // leave out to run for <value> seconds
            .end(() -> mechanism.finish());
}
```

- `tag(name)` with no action works like `stopTag`, but the robot keeps driving.
- A `start`/`loop`/`until`/`end` tag with a value of 0 or less is skipped.
- `velocity` (percent of max speed) and `pause` (seconds) are built in.
- Tags on a route that are not in your list show up in telemetry as ignored.
- To make a tag selectable in the planner, add one line for it to the planner's `config.js`.

To upload, pick your OpMode in the planner and press Init & upload, then Start. To run a saved route
with no laptop, export it to `TeamCode/src/main/assets` and override `routeFile()`.
