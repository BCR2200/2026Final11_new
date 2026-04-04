package frc.robot.commands.auto;

import java.util.List;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;

public class SweepDriver extends AutoCommand {

    private PathPlannerPath path;
    
    public SweepDriver(RobotContainer robotContainer, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
        path = AutoBuildingBlocks.loadPathOrThrow("SweepDriver.1");

        addCommands(
            new WaitCommand(0.01),
            AutoBuildingBlocks.autoStep("PATH"),
            // TODO: MAKE THE MAX SPEED to drive to the path HIGHER LATER
            Commands.race(
                drivetrain.applyRequest(() -> robotContainer.driveToPose(getProperFlippedStartingPose(), 1, 6.0)),
                new WaitUntilCommand(() -> robotContainer.atTargetPos(getProperFlippedStartingPose(), 0.1))
            ),
            AutoBuildingBlocks.followPathCommand(path, drivetrain)
        );
    }

    @Override
    List<Pose2d> getAllRawPathPoses() {
        return path.getPathPoses();
    }

    @Override
    Pose2d getRawStartingPose() {
        return path.getStartingHolonomicPose().orElseThrow();
    }

}
