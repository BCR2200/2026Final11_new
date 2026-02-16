package frc.robot;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

/**
 * Custom Epilogue logger for PIDMotor.
 * Logs motor telemetry data during Epilogue's logging phase (offset from control loop).
 */
@CustomLoggerFor(PIDMotor.class)
public class PIDMotorLogger extends ClassSpecificLogger<PIDMotor> {

    public PIDMotorLogger() {
        super(PIDMotor.class);
    }

    @Override
    public void update(EpilogueBackend backend, PIDMotor motor) {
        backend.log("Position", motor.getPosition());
        backend.log("Velocity", motor.getVelocity());
        backend.log("Current", motor.getCurrent());
        backend.log("Target", motor.target);
    }
}
