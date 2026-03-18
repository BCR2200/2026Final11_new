package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ExtraMath;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.*;

public class DriveToOutpostCmd extends Command {

    /**
     * P value (proportional output) used in driveToPose.
     */
    @NotLogged
    public static final double TRANSLATION_P = 6.0;

    public Pose2d targetOutpost;

    public static final Pose2d OUTPOST_RED = new Pose2d(
        Distance.ofBaseUnits(15.22, Meters), // TODO
        Distance.ofBaseUnits(4.73, Meters), // TODO
        Rotation2d.k180deg
    );
    public static final Pose2d OUTPOST_BLUE = new Pose2d(
        Distance.ofBaseUnits(15.22, Meters), // TODO
        Distance.ofBaseUnits(4.73, Meters), // TODO
        Rotation2d.k180deg
    );

    private final CommandSwerveDrivetrain drivetrain;
    private final RobotContainer robotContainer;

    public DriveToOutpostCmd(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.drivetrain = robotContainer.drivetrain;
        addRequirements(drivetrain);
    }

    public double getXToTarget(Pose2d targetPose) {
        Pose2d robotPose2d = drivetrain.getState().Pose;
        return targetPose.getX() - robotPose2d.getX();
    }

    public double getYToTarget(Pose2d targetPose) {
        Pose2d robotPose2d = drivetrain.getState().Pose;
        return targetPose.getY() - robotPose2d.getY();
    }

    /**
     * Returns true if the robot is at the specified postion, false otherwise
     * @param targetPos the target pos
     * @param threashold the threashold that the robot can be within to count as there
     * @return if robot is at the targetPos
     */
    public boolean atTargetPos(Pose2d targetPos, double threashold) {
        return getDistanceToTarget(targetPos) < threashold;
    }

    public double getDistanceToTarget(Pose2d targetPose) {
        Pose2d robotPose = drivetrain.getState().Pose;
        return robotPose.getTranslation().getDistance(targetPose.getTranslation());
    }

    private SwerveRequest.FieldCentricFacingAngle driveToPose(Pose2d target) {
        return robotContainer.driveFCFAVelocityMode.withVelocityX(ExtraMath.clampedDeadzone(getXToTarget(target)*TRANSLATION_P, 1, 0.0001))
                .withVelocityY(ExtraMath.clampedDeadzone(getYToTarget(target)*TRANSLATION_P, 1, 0.0001))
                .withTargetDirection(target.getRotation())
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    }

    @Override
    public void initialize() {
        if (Robot.alliance == DriverStation.Alliance.Red) {
            targetOutpost = OUTPOST_RED;
        }
        else {
            targetOutpost = OUTPOST_BLUE;
        }
    }

    @Override
    public void execute() {
        drivetrain.setControl(driveToPose(targetOutpost));
    }

    @Override
    public void end(boolean interrupted) {}
}
