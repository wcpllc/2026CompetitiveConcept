# Reading a Drivetrain Log

This guide is for anyone on the team, including people who have never opened a robot log
before. It explains how to get a log off the robot, open it, and use it to figure out which
part of the drivetrain is causing a problem.

The big idea: **all four swerve modules are told to do the same kind of thing. The one that
behaves differently is the broken one.**

## 1. What is a log?

While the robot runs, the code writes down hundreds of numbers 50 times a second: where the
robot thinks it is, what the driver's sticks are doing, how much current each motor is pulling,
whether any device reported a fault, and so on. That file is the log. It lets us replay a
practice session on a laptop and see exactly what happened, even if nobody was watching.

The robot writes two kinds of log:

| Log | What it holds | Where it goes |
|---|---|---|
| `.wpilog` (AdvantageKit) | Everything the robot code chose to record. This is the one this guide is about. | USB stick in the roboRIO, or `/home/lvuser/logs` on the roboRIO if no stick is plugged in |
| `.hoot` (CTRE) | Every signal from every motor and sensor on the CAN bus. Very detailed, less friendly. | Same places |

If no USB stick is plugged in, the dashboard shows a yellow alert saying logs are going to the
roboRIO instead. That still works, but the roboRIO has limited space, so delete old logs from
it every few weeks.

## 2. Getting the log onto a laptop

**If there is a USB stick in the roboRIO:** disable the robot, pull the stick, plug it into
the laptop. The `.wpilog` files are at the top level of the stick.

**If there is no USB stick:** connect the laptop to the robot (USB cable to the roboRIO, or
the practice radio), open AdvantageScope, choose **File > Download Logs**, and enter the
robot address (`10.95.84.2`). In AdvantageScope's preferences, set the roboRIO log folder to
`/home/lvuser/logs`.

Log files are named by date and time, so the newest one is the one from the session you just
ran. Rename it to something useful before you forget, like
`2026-10-31-practice-frontleft-shudder.wpilog`.

## 3. Opening the log

AdvantageScope is installed with WPILib. Open it, drag the `.wpilog` file into the window, and
you will see a tree of names on the left. Those names are the things the robot recorded. Click
the **Line Graph** tab at the top, then drag names from the tree onto the graph.

The timeline at the bottom shows when the robot was disabled, in auto, or in teleop. Click
anywhere on it to jump to that moment.

## 4. The names you care about

In AdvantageScope, everything the robot code recorded shows up under `RealOutputs`, so the
full path to a name below is `RealOutputs/Drive/...`. The raw controller data is under
`DriverStation` at the top level, not under `RealOutputs`.

Everything for the drivetrain lives under `Drive`. Each module has its own folder:
`Drive/Modules/FrontLeft`, `FrontRight`, `BackLeft`, `BackRight`.

| Name (inside each module folder) | What it means | Normal looks like |
|---|---|---|
| `TargetAngleDegrees` / `MeasuredAngleDegrees` | Where the wheel was told to point, and where it actually points | The two lines sit on top of each other |
| `TargetSpeedMetersPerSec` / `MeasuredSpeedMetersPerSec` | How fast the wheel was told to spin, and how fast it did | Lines match, with a small lag |
| `Drive/SupplyCurrentAmps` | How hard the drive motor is working | 0 to 2 A sitting still, 10 to 40 A accelerating, spikes briefly when you push on something |
| `Steer/SupplyCurrentAmps` | How hard the steer motor is working | Under 10 A, brief bumps when wheels turn |
| `Drive/TempCelsius` / `Steer/TempCelsius` | Motor temperature | Under 70 C. Above 90 C the motor protects itself and gets weak |
| `Drive/VoltageVolts` / `Steer/VoltageVolts` | Voltage the motor controller is actually applying | Moves with the sticks |
| `Steer/ErrorDegrees` | How many degrees the wheel is away from its target angle | Near 0, with short blips when the target jumps |
| `Encoder/AbsoluteDegrees` | The CANcoder's reading of the wheel angle | Changes smoothly as the wheel turns |
| `Encoder/MagnetOk` | Whether the CANcoder can see its magnet properly | `true` the whole time |
| `Drive/Connected`, `Steer/Connected`, `Encoder/Connected` | Whether the device is answering on the CAN bus | `true` the whole time |
| `Drive/Faults`, `Steer/Faults`, `Encoder/Faults` | Names of any faults the device is reporting right now | Empty |
| `HasFault` | True if anything in this module is faulted or not responding | `false` |

Outside the module folders:

| Name | What it means |
|---|---|
| `Drive/AnyModuleFault` | True if any module has any problem. Check this first. |
| `Drive/Input/Forward`, `Left`, `Rotation` | The driver's stick input after deadband and curve, from -1 to 1. This is what the code actually sent to the drivetrain. |
| `Drive/Input/State` | What the drive command was doing: `IDLING`, `DRIVING_WITH_MANUAL_ROTATION`, or `DRIVING_WITH_LOCKED_HEADING` |
| `DriverStation/Joystick0/...` | The raw Xbox controller axes and buttons, before the code touches them |
| `BatteryVoltage` | Battery voltage. Below about 7 V the roboRIO browns out and motors cut out |
| `Power/TotalCurrent` | Total current from the battery |
| `CAN/BusOffCount`, `CAN/ReceiveErrorCount` | CAN bus wiring health. Should stay at 0 |

## 5. How to find the broken part

Start at the moment the problem happened. If the driver said "it shuddered when I turned
left near the end of practice," go to the last minute of teleop.

**Step 1. Check `Drive/AnyModuleFault`.** If it went true, open each module's `HasFault`
to find which one, then read its `Faults` text. That often answers the question outright.
The dashboard shows the same thing as a red alert while the robot is on.

**Step 2. Put the same number from all four modules on one graph.** For example, drag
`Drive/SupplyCurrentAmps` from all four module folders onto the line graph. Three lines will
look alike and one will not. That is your module.

**Step 3. Use the table below to turn what you see into a suspect.**

| What you see | What it probably means | What to check on the robot |
|---|---|---|
| `MeasuredAngleDegrees` oscillates, lags, or sits at the wrong angle on one module while the others track | Steer feedback problem | That module's CANcoder, its magnet, and its CAN wire. Look at `Encoder/MagnetOk` and `Steer/Faults` for `RemoteSensorDataInvalid` |
| `Encoder/MagnetOk` goes false, or `Encoder/Faults` shows `BadMagnet` | The CANcoder cannot see its magnet | The magnet has shifted or fallen out of the steer shaft, or the CANcoder is mounted too far away |
| Big `Steer/ErrorDegrees` and high `Steer/SupplyCurrentAmps` | The steer motor is fighting to turn the wheel and losing | Something is binding: debris in the module, a bent part, a stripped gear |
| Big `Steer/ErrorDegrees` and near-zero `Steer/SupplyCurrentAmps` | The steer motor is not responding | Power or CAN wiring to that motor, or the motor itself. Check `Steer/Connected` |
| `MeasuredSpeedMetersPerSec` well below target with high `Drive/SupplyCurrentAmps` | The drive motor is working hard but the wheel is not turning | Drive gear binding, a seized bearing, or a dead motor fighting its brake |
| `MeasuredSpeedMetersPerSec` matches target, low current, but the robot did not actually move that way | The wheel is spinning but not gripping | Worn tread, a loose wheel, or the robot was pushed |
| `Drive/SupplyCurrentAmps` is flat zero when the others are busy | That drive motor is doing nothing | Power or CAN wiring to that motor. Check `Drive/Connected` |
| Any `Connected` flickers false | That device is dropping off the CAN bus | The CAN wire and its connector for that device. Also check `CAN/ReceiveErrorCount` |
| `Drive/TempCelsius` or `Steer/TempCelsius` above 90 C | The motor is overheating and limiting itself | Let it cool, then look for what is making it work so hard |
| `BatteryVoltage` dips below 7 V when the problem happens | Brownout. Everything cuts out at once, not one module | The battery, the main breaker, or the battery leads. Not a module problem |
| `DriverStation/Joystick0` axes drop to zero or jump with no hand movement | The controller or its USB cable | Swap the controller. Check that `Drive/Input` follows the raw axis |
| Raw joystick axes look fine but `Drive/Input` does not match them | A code problem, not hardware | Talk to a programmer and bring the log |

**Step 4. Write it down.** Before touching anything on the robot, write in the build log
which module, which signal, and what time in the log. Then fix one thing and drive again.
If the next log looks clean, you found it. If not, you ruled one thing out and the log will
show you the next suspect.

## 6. Keep in mind

- Compare, don't stare. One graph with four modules on it tells you more than one module's
  graph studied for ten minutes.
- Currents spike normally when the robot accelerates or hits something. A spike on all four
  modules at once is driving. A spike on one module by itself is a problem.
- Faults clear when the robot restarts. If you suspect a fault, look at the log from the
  session where it happened, not a fresh boot.
- Ask. Logs are new to most of the team. Reading one together is the fastest way to learn.
