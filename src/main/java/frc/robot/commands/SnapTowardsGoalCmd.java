package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;
public class SnapTowardsGoalCmd extends Command {

    private RobotContainer robotContainer;
    
    public SnapTowardsGoalCmd(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    @Override
    public void execute() {
        robotContainer.shootingAtHub = true;
    }

    @Override
    public void end(boolean interrupted) {
        robotContainer.shootingAtHub = false;
    }

}
