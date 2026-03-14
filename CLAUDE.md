# CLAUDE.md - AI Assistant Guide for FRC 2026 Robot Code

## Project Overview

This is an FRC (FIRST Robotics Competition) robot project using WPILib's command-based framework with CTRE Phoenix 6 motor controllers. The robot has a swerve drivetrain, three shooter mechanisms, an intake system, and a climber.

## Build & Validation

(Update the java home path to match your installation)

```bash
./gradlew build -Dorg.gradle.java.home="/home/hugo/wpilib/2026/jdk"
```

Always run this after making changes to verify the code compiles.

## Project Structure

```
src/main/java/frc/robot/
├── Robot.java              # Entry point, lifecycle methods, periodic updates
├── RobotContainer.java     # Wires subsystems, commands, and controller bindings
├── Constants.java          # CAN IDs, ports, hardware constants
├── PIDMotor.java           # Helper class wrapping TalonFX with PID/Motion Magic
├── Interpolator.java       # Linear interpolation utility for distance-based values
├── LinearActuator.java     # PWM linear actuator control
├── LimelightHelpers.java   # Vision processing utilities
├── subsystems/             # Robot mechanism subsystems
│   ├── ShooterSubsystem.java
│   ├── IntakeSubsystem.java
│   ├── FloorFeedSubsystem.java
│   └── ClimbSubsystem.java
├── commands/               # Robot actions
│   ├── ShootAt.java
│   ├── BlendAdamModeCmd.java
│   ├── ClimbCommand.java
│   └── auto/               # Autonomous routines
└── drive/                  # Swerve drivetrain (CTRE generated)
    ├── CommandSwerveDrivetrain.java
    ├── TunerConstantsComp.java
    └── TunerConstantsPrac.java
```

## Key Patterns

### Subsystem Pattern

Subsystems represent physical mechanisms. They extend `SubsystemBase` and have:
- State variables (e.g., `isIntaking`, `isShooting`)
- Motor controllers (typically `PIDMotor` instances)
- Sensors (beam breaks, encoders)
- `periodic()` method called every 20ms for control logic

```java
@Logged
public class ExampleSubsystem extends SubsystemBase {
    private boolean isRunning = false;
    public PIDMotor motor;
    
	/* Constructor omitted for brevity */

    public void setIsRunning(boolean running) { this.isRunning = running; }

    @Override
    public void periodic() {
        if (isRunning) {
            motor.setVelocityTarget(50);
        } else {
            motor.setPercentOutput(0);
        }
    }
}
```

### Command Pattern

Commands are actions that use subsystems. They extend `Command` and have:
- `initialize()` - Called once when command starts
- `execute()` - Called every 20ms while running
- `end(boolean interrupted)` - Cleanup when command ends
- `isFinished()` - Return true to end the command
- `addRequirements()` - Declare which subsystems are used (prevents conflicts)

```java
public class ExampleCommand extends Command {
    public ExampleCommand(Subsystem subsystem) {
        addRequirements(subsystem);
    }
    
    @Override
    public void init() { /* Runs once when command starts */ }

    @Override
    public void execute() { /* ... */ }
  
	  @Override
	  public void end(boolean interrupted) { /* Runs once when command ends */ }

    @Override
    public boolean isFinished() { return false; } // Runs until interrupted
}
```

### Trigger/Binding Pattern

Controller buttons are bound to commands in `RobotContainer`:

```java
// Run command while button held
driverController.leftBumper().whileTrue(new SomeCommand());

// Run instant command on button press
driverController.a().onTrue(new InstantCommand(() -> subsystem.doThing()));
```

## Key Classes

### PIDMotor

Wrapper around `TalonFX` providing simplified PID and Motion Magic control.

```java
// Creation
PIDMotor motor = PIDMotor.makeMotor(deviceID, "name", p, i, d, s, v, a, maxV, maxA, maxJerk);

// Position control (Motion Magic)
motor.setTarget(position);
motor.setTarget(position, velocity, acceleration);  // Dynamic motion

// Velocity control
motor.setVelocityTarget(rps);

// Direct control
motor.setPercentOutput(0.5);  // -1.0 to 1.0

// Current limits
motor.setStatorCurrentLimit(60);  // Motor winding current
motor.setSupplyCurrentLimit(40);  // Battery draw current

// State
motor.getPosition();   // rotations
motor.getVelocity();   // rotations per second
motor.atPosition(epsilon);
motor.atVelocity(threshold);
```

### Interpolator

Linear interpolation for distance-based parameters (hood angle, shooter speed).

```java
Interpolator interp = new Interpolator(
    new double[] { 1.0, 2.0, 3.0 },  // distances (meters)
    new double[] { 0.1, 0.5, 0.9 }   // corresponding values
);
double value = interp.interpolate(1.5);  // Returns 0.3
```

### ShooterSubsystem

The robot has three shooters: John (left), Jawbreaker (middle), Taylor (right). Each has:
- Shooter motor (flywheel)
- Feeder motor (feeds balls into shooter)
- Beam break sensor (detects ball presence)
- Linear actuator (hood angle)

Key methods:
- `setIsShooting(boolean)` - Spin up flywheel
- `setIsFeeding(boolean)` - Feed balls into shooter
- `isShooterAtSpeed()` - Check if flywheel ready
- `isBeamBroken()` - Check if ball present
- `setActuatorPositionViaInterpolatedValue(distance)` - Auto-adjust hood

### RobotContainer

Central wiring class. Contains:
- All subsystem instances
- Controller bindings (`configureBindings()`)
- Autonomous chooser
- Target tracking (`targetHub`, `passTarget`, `compensatedTargetHub`)
- Helper methods (`getDistanceToTarget()`, `getDegreesToTarget()`, `isOutsideAllianceZone()`)

## Hardware Configuration

- **Drivetrain**: CTRE Swerve (4 modules), Pigeon 2 IMU (CAN ID 23)
- **Shooters**: 3x (shooter motor + feeder motor + beam break + linear actuator)
- **Intake**: Intake motor (ID 7) + Tilt motor (ID 8)
- **Floor Feed**: Single motor (ID 9) feeding all three shooters
- **Climber**: Single motor (ID 10)
- **Vision**: Limelight cameras ("limelight-shooter", "limelight-intake")

## Alliance & Field Awareness

- `Robot.alliance` - Current alliance (Red/Blue), updated from DriverStation
- `RobotContainer.isOutsideAllianceZone()` - Determines if robot should pass or shoot at hub
- `RobotContainer.calculateCompensatedTargetHub()` - Adjusts aim based on robot velocity

## Logging

Uses WPILib Epilogue for automatic telemetry:
- `@Logged` on classes/fields for automatic logging
- `@NotLogged` to exclude fields
- Logged to "Telemetry" root in NetworkTables

## Common Tasks

### Adding a New Subsystem

1. Create class in `subsystems/` extending `SubsystemBase`
2. Add `@Logged` annotation
3. Create motors with `PIDMotor.makeMotor()`
4. Implement `periodic()` for control logic
5. Instantiate in `RobotContainer`

### Adding a New Command

1. Create class in `commands/` extending `Command`
2. Accept required subsystems in constructor
3. Call `addRequirements()` with all used subsystems
4. Implement `execute()` and `end()`
5. Bind to controller in `RobotContainer.configureBindings()`

### Adding an Autonomous Routine

1. Create class in `commands/auto/` extending `AutoCommand`
2. Add to `autoChooser` in `RobotContainer` constructor
