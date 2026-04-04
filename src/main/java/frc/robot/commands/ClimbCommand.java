package frc.robot.commands;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.ClimbSubsystem;

import static edu.wpi.first.units.Units.*;

public class ClimbCommand extends Command {

    /**
     * P value (proportional output) used in driveToPose.
     */
    @NotLogged
    public static final double TRANSLATION_P = 6.0;

    public static final Pose2d BLUE_L_CLIMB_INITIAL = new Pose2d(
            Distance.ofBaseUnits(1.535, Meters),
            Distance.ofBaseUnits(4.23, Meters),
            Rotation2d.kZero
    );
    public static final Pose2d BLUE_L_CLIMB_FINAL = new Pose2d(
            Distance.ofBaseUnits(1.36, Meters),
            Distance.ofBaseUnits(4.23, Meters),
            Rotation2d.kZero
    );
    public static final Pose2d BLUE_R_CLIMB_INITIAL = new Pose2d(
            Distance.ofBaseUnits(1.535, Meters),
            Distance.ofBaseUnits(3.382, Meters),
            Rotation2d.kZero
    );
    public static final Pose2d BLUE_R_CLIMB_FINAL = new Pose2d(
            Distance.ofBaseUnits(1.346, Meters),
            Distance.ofBaseUnits(3.382, Meters),
            Rotation2d.kZero
    );
    public static final Pose2d RED_L_CLIMB_INITIAL = new Pose2d(
            Distance.ofBaseUnits(14.9, Meters),
            Distance.ofBaseUnits(3.79, Meters),
            Rotation2d.k180deg
    );
    public static final Pose2d RED_L_CLIMB_FINAL = new Pose2d(
            Distance.ofBaseUnits(15.16, Meters),
            Distance.ofBaseUnits(3.79, Meters),
            Rotation2d.k180deg
    );
    public static final Pose2d RED_R_CLIMB_INITIAL = new Pose2d(
            Distance.ofBaseUnits(14.9, Meters),
            Distance.ofBaseUnits(4.64, Meters),
            Rotation2d.k180deg
    );
    public static final Pose2d RED_R_CLIMB_FINAL = new Pose2d(
            Distance.ofBaseUnits(15.16, Meters),
            Distance.ofBaseUnits(4.64, Meters),
            Rotation2d.k180deg
    );

    public Pose2d targetClimbInitial = RED_R_CLIMB_INITIAL;
    public Pose2d targetClimbFinal = RED_R_CLIMB_FINAL;

    private final CommandSwerveDrivetrain drivetrain;
    private final ClimbSubsystem climberSubsystem;
    private final RobotContainer robotContainer;
    private final boolean isOnRight;
    private boolean goneToInitialPos = false;

    public ClimbCommand(RobotContainer robotContainer, boolean isRight) {
        this.robotContainer = robotContainer;
        this.drivetrain = robotContainer.drivetrain;
        this.climberSubsystem = robotContainer.climberSubsystem;
        this.isOnRight = isRight;
        addRequirements(drivetrain, climberSubsystem);
    }

    @Override
    public void initialize() {
        goneToInitialPos = false;
        if (this.isOnRight) {
            this.setupClimbR();
        } else {
            this.setupClimbL();
        }
    }

    @Override
    public void execute() {
        if (robotContainer.atTargetPos(targetClimbFinal, 0.015)) { // At final
            climberSubsystem.climb();
            drivetrain.setControl(robotContainer.driveFC.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
        } else if (robotContainer.atTargetPos(targetClimbInitial, 0.06) || goneToInitialPos) { // Past initial
            goneToInitialPos = true;
            drivetrain.setControl(robotContainer.driveToPose(targetClimbFinal, 1, TRANSLATION_P));
        } else { // Not at initial
            drivetrain.setControl(robotContainer.driveToPose(targetClimbInitial, 1, TRANSLATION_P));
        }
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.goHome();
    }

    public ClimbCommand setupClimbL() {
        if (Robot.alliance == DriverStation.Alliance.Red) {
            targetClimbFinal = RED_L_CLIMB_FINAL;
            targetClimbInitial = RED_L_CLIMB_INITIAL;
        } else {
            targetClimbFinal = BLUE_L_CLIMB_FINAL;
            targetClimbInitial = BLUE_L_CLIMB_INITIAL;
        }
        return this;
    }
    public ClimbCommand setupClimbR() {
        if (Robot.alliance == DriverStation.Alliance.Red) {
            targetClimbFinal = RED_R_CLIMB_FINAL;
            targetClimbInitial = RED_R_CLIMB_INITIAL;
        } else {
            targetClimbFinal = BLUE_R_CLIMB_FINAL;
            targetClimbInitial = BLUE_R_CLIMB_INITIAL;
        }
        return this;
    }
}
