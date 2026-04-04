package frc.robot.commands.auto;

import java.util.List;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;

public class SweepDriver extends AutoCommand {

    private PathPlannerPath pathRight;
    private PathPlannerPath pathLeft;
    private PathPlannerPath path;
    
    public SweepDriver(RobotContainer robotContainer, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
        pathRight = AutoBuildingBlocks.loadPathOrThrow("SweepDriverRight.1");
        pathLeft = AutoBuildingBlocks.loadPathOrThrow("SweepDriverLeft.1");
        path = isOnRightSide(drivetrain) ? pathRight : pathLeft;

        addCommands(
            new WaitCommand(0.01),
            AutoBuildingBlocks.autoStep("PATH"),
            Commands.race(
                drivetrain.applyRequest(() -> robotContainer.driveToPose(getProperFlippedStartingPose(), 2, 6.0)),
                new WaitUntilCommand(() -> robotContainer.atTargetPos(getProperFlippedStartingPose(), 0.1))
            ),
            AutoBuildingBlocks.followPathCommand(path, drivetrain)
        );
    }

    private boolean isOnRightSide(CommandSwerveDrivetrain drivetrain) {
        if (Robot.alliance == DriverStation.Alliance.Blue) {
            if (drivetrain.getState().Pose.getY() < 4) {
                return true;
            }
            else {
                return false;
            }
        }
        else {
            if (drivetrain.getState().Pose.getY() > 4) {
                return true;
            }
            else {
                return false;
            }
        }
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
