package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

@Logged
public class FloorFeedSubsystem extends SubsystemBase {

    // Logged automatically by Epilogue
    private double motorSpeedCentre = 20; // in rps
    private boolean isFeeding = false;
    @Logged
    private boolean needsToRun = false;

    @Logged(name = "Motor")
    private PIDMotor motor;

    @NotLogged
    private static final double MAX_RPS = 140.0;
    @NotLogged
    private static final double RPS_STEP = 4.0;
    @NotLogged
    private static final double SECONDS_TO_HIGH_POINT = 0.2;
    @NotLogged
    private static final double TIME_FACTOR_TO_LOW_POINT = 4;
    @NotLogged
    private static final double SPEED_CHANGE_FACTOR = 1.0;
    @NotLogged
    private static final double PARAM_P = 0.11;
    @NotLogged
    private static final double PARAM_I = 0.0;
    @NotLogged
    private static final double PARAM_D = 0.0;
    @NotLogged
    private static final double PARAM_S = 0.25;
    @NotLogged
    private static final double PARAM_V = 1.2;
    @NotLogged
    private static final double PARAM_A = 0.01;
    @NotLogged
    private static final double PARAM_MV = MAX_RPS;
    @NotLogged
    private static final double PARAM_MA = MAX_RPS * 2;
    @NotLogged
    private static final double PARAM_MJ = 0.00;

    @NotLogged
    private ShooterSubsystem[] shooters;

    public FloorFeedSubsystem(int currentLimit, ShooterSubsystem... shooters) {
        this.shooters = shooters;
        motor = PIDMotor.makeMotor(Constants.FLOOR_FEED_MOTOR_ID, "Floor Feed",
                PARAM_P, PARAM_I, PARAM_D, PARAM_S, PARAM_V, PARAM_A, PARAM_MV, PARAM_MA, PARAM_MJ);
        motor.setInverted(InvertedValue.Clockwise_Positive);
        motor.setCurrentLimit(currentLimit);
        motor.setIdleCoastMode();
    }

    public boolean getIsFeeding() {
        return isFeeding;
    }
    public void setIsFeeding(boolean feeding) {
        isFeeding = feeding;
    }

    public double getMotorSpeedCentre() {
        return motorSpeedCentre;
    }
    public void setMotorSpeedCentre(double speed) {
        motorSpeedCentre = speed;
    }

    public void incrementMotorSpeed() {
        motorSpeedCentre += RPS_STEP;
        motorSpeedCentre = Math.min(motorSpeedCentre, MAX_RPS);
    }
    public void decrementMotorSpeed() {
        motorSpeedCentre -= RPS_STEP;
        motorSpeedCentre = Math.max(motorSpeedCentre, -MAX_RPS);
    }

    public void updateParameters(){
        motorSpeedCentre = SmartDashboard.getNumber("Floor Feed motor speed centre", motorSpeedCentre);
        motor.fetchPIDFFromDashboard();
    }

    private double getVelocityAtTime(double t) {
        double highSlope, lowSlope, highPoint, lowPoint, speedDelta;

        // find parameters
        highPoint = motorSpeedCentre * (1 + (SPEED_CHANGE_FACTOR / 2));
        lowPoint = motorSpeedCentre * (1 - (SPEED_CHANGE_FACTOR / 2));
        speedDelta = highPoint - lowPoint;

        highSlope = speedDelta / SECONDS_TO_HIGH_POINT;
        lowSlope = -(speedDelta / (SECONDS_TO_HIGH_POINT * TIME_FACTOR_TO_LOW_POINT));

        // low-high takes SECONDS_TO_HIGH_POINT, high-low takes SECONDS_TO_HIGH_POINT * TIME_FACTOR_TO_LOW_POINT
        double x = t % (SECONDS_TO_HIGH_POINT + SECONDS_TO_HIGH_POINT * TIME_FACTOR_TO_LOW_POINT);
        // find value
        if (x < SECONDS_TO_HIGH_POINT) {
            // upwards slope
            return highSlope * x + lowPoint;
        } else {
            // downwards slope
            return lowSlope * (x - SECONDS_TO_HIGH_POINT) + highPoint;
        }

    }

    @Override
    public void periodic() {
        // Control logic only - telemetry handled by Epilogue

        // Check if any shooter needs the floor to run (or if isFeeding is manually set to true)
        needsToRun = false;
        for (ShooterSubsystem shooter : shooters) {
            needsToRun |= shooter.needsFloorFeed();
            if (needsToRun) break;
        }

        if (needsToRun) {
            // Use FPGA timestamp for consistent timing
            motor.setVelocityTarget(getVelocityAtTime(Timer.getFPGATimestamp()));
        } else {
            motor.setPercentOutput(0);
        }
    }

}
