package frc.robot.commands.auto;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootAt;
import frc.robot.drive.CommandSwerveDrivetrain;

public class RightOutpost extends AutoCommand{
    private final PathPlannerPath path1;
    private final PathPlannerPath path2;
    private final PathPlannerPath path3;

    public RightOutpost (RobotContainer robot, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
        path1 = AutoBuildingBlocks.loadPathOrThrow("RightOutpost.1");
        path2 = AutoBuildingBlocks.loadPathOrThrow("RightOutpost.2");
        path3 = AutoBuildingBlocks.loadPathOrThrow("RightOutpost.3");
        
        addCommands(
          new WaitCommand(0.01),
          AutoBuildingBlocks.autoStep("PATH 1"),
          AutoBuildingBlocks.followPathCommand(path1, drivetrain),
          new WaitCommand(5),
          AutoBuildingBlocks.autoStep("PATH 2"),
          AutoBuildingBlocks.followPathCommand(path2, drivetrain),
          AutoBuildingBlocks.autoStep("PATH 3"),
          AutoBuildingBlocks.followPathCommand(path3, drivetrain),
          new ShootAt(robot)
        );
    }

    
  @Override
  List<Pose2d> getAllRawPathPoses() {
    return Stream
        .of(path1.getPathPoses(), path2.getPathPoses(), path3.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getRawStartingPose() {
    return path1.getStartingHolonomicPose().orElseThrow();
  }
}
