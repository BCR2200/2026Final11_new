package frc.robot.commands.auto;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ShootAt;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;

public class RightOutpostAroundClimber extends AutoCommand {
  private final PathPlannerPath path1;
  private final PathPlannerPath path2;
  private final PathPlannerPath path3;

  public RightOutpostAroundClimber(RobotContainer robot, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
    path1 = AutoBuildingBlocks.loadPathOrThrow("RightOutpostAroundClimber.1");
    path2 = AutoBuildingBlocks.loadPathOrThrow("RightOutpostAroundClimber.2");
    path3 = AutoBuildingBlocks.loadPathOrThrow("RightOutpostAroundClimber.3");

    addCommands(
      new WaitCommand(0.01),
      AutoBuildingBlocks.autoStep("PATH 1"),
      AutoBuildingBlocks.followPathCommand(path1, drivetrain),
      new WaitCommand(1.5),
      AutoBuildingBlocks.autoStep("PATH 2"),
      AutoBuildingBlocks.followPathCommand(path2, drivetrain),
      AutoBuildingBlocks.autoStep("PATH 3"),
      AutoBuildingBlocks.followPathCommand(path3, drivetrain),
      Commands.race(
        new ShootAt(robot),
        Commands.sequence(
          new WaitCommand(4), // Wait before intake up
          new InstantCommand(() -> robot.intakeSubsystem.setTiltPosition(IntakeSubsystem.tiltHalfExtensionPos)),
          new WaitCommand(2) // Wait to finish shooting
        )
      ),
      new ClimbCommand(robot, true));
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
