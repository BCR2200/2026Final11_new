package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.drive.CommandSwerveDrivetrain;

public class SnapTowardsGoalCmd extends Command {

    private CommandSwerveDrivetrain drivetrain;
    
    public SnapTowardsGoalCmd(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        // botHubPose is in field space
        // if the forward direction is 180deg, we are on red; otherwise we are blue
        Pose2d hubPose = drivetrain.getOperatorForwardDirection().equals(Rotation2d.k180deg) ? Robot.RED_HUB : Robot.BLUE_HUB;
        Pose2d currentBotPose = drivetrain.getState().Pose;
        Transform2d botToHubTransform = hubPose.minus(currentBotPose);

        double angle = Math.atan(botToHubTransform.getX() / botToHubTransform.getY());
        drivetrain.applyRequest(() -> 
            new SwerveRequest.FieldCentricFacingAngle()
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                .withTargetDirection(Rotation2d.fromRadians(angle))
        );
    }

}
