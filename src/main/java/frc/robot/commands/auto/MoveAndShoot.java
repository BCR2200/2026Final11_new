package frc.robot.commands.auto;

import frc.robot.input.SnapButton;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MoveAndShoot extends SequentialCommandGroup {

  public MoveAndShoot (PathPlannerPath path) {
    addCommands(
      new ParallelCommandGroup(
          AutoBuildingBlocks.followPathCommand(path)
          //TODO Add Shoot CMD and alter parameters for the CMD
        )
    );
  }
}
