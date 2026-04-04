package frc.robot.commands;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;

public class DriveToOutpostCmd extends Command {

    /**
     * P value (proportional output) used in driveToPose.
     */
    @NotLogged
    public static final double TRANSLATION_P = 6.0;

    public Pose2d finalTarget;
    public Pose2d initialTarget;
    public boolean goneToInitialPos = false;

    private final CommandSwerveDrivetrain drivetrain;
    private final RobotContainer robotContainer;

    public DriveToOutpostCmd(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.drivetrain = robotContainer.drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        goneToInitialPos = false;
        if (Robot.alliance == DriverStation.Alliance.Red) {
            initialTarget = Constants.OUTPOST_RED_INITIAL;
            finalTarget = Constants.OUTPOST_RED_FINAL;
        }
        else {
            initialTarget = Constants.OUTPOST_BLUE_INITIAL;
            finalTarget = Constants.OUTPOST_BLUE_FINAL;
        }
    }

    @Override
    public void execute() {
        if (robotContainer.atTargetPos(finalTarget, 0.015)) { // At final
            drivetrain.setControl(robotContainer.driveFC.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
        } 
        else if (robotContainer.atTargetPos(initialTarget, 0.06) || goneToInitialPos) { // Past initial
            goneToInitialPos = true;
            drivetrain.setControl(robotContainer.driveToPose(finalTarget, 0.6, TRANSLATION_P));
        } 
        else { // Not at initial
            drivetrain.setControl(robotContainer.driveToPose(initialTarget, 2, TRANSLATION_P));
        }
    }

    @Override
    public void end(boolean interrupted) {
        goneToInitialPos = false;
    }
}
