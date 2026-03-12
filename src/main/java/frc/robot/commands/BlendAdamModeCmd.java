package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ExtraMath;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;

public class BlendAdamModeCmd extends Command {

    private final RobotContainer robot;

    private CommandSwerveDrivetrain drivetrain;
    private Pose2d currentPose2d;

    // TODO correct these values for the errors on the blue side, and for a rotation buffer

    // in meters
    private double blueHUBLine = 5.7;
    // TODO FIND VALUE
    private double redHUBLine = 11.6;

    // left/right from blue's perspective
    private double wallLeftLine = 7.590;
    private double wallRightLine = 0.319;

    public BlendAdamModeCmd(RobotContainer robot) {
        addRequirements(robot.drivetrain);
        this.drivetrain = robot.drivetrain;
        this.robot = robot;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        /*
         * If (in a corner, allow driver X and Y)
         * If (On one of the X lines, only allow Y)
         * If (On one of the Y lines, only allow X)
         * If (Not on either, free reign)
         */

        currentPose2d = drivetrain.getState().Pose;
        var x = currentPose2d.getX();
        var y = currentPose2d.getY();

        // Red/blue lines are parallel to Y, compare to X
        // Side lines are parallel to X, compare to Y
        // wallSomethingLine is from blue's perspective
        boolean passedRedHubLine = x > redHUBLine;
        boolean passedBlueHubLine = x < blueHUBLine;
        boolean passedWallLFromBlueLine = y > wallLeftLine;
        boolean passedWallRFromBlueLine = y < wallRightLine;

        double velocityX = -RobotContainer.driverY * RobotContainer.MaxSpeed;
        double velocityY = -RobotContainer.driverX * RobotContainer.MaxSpeed;

        if (passedRedHubLine) {
            velocityX = Math.min(velocityX, 0);
        } else if (passedBlueHubLine) {
            velocityX = Math.max(velocityX, 0);
        }

        if (passedWallLFromBlueLine) {
            velocityY = Math.min(velocityY, 0); 
        } else if (passedWallRFromBlueLine) {
            velocityY = Math.max(velocityY, 0);
        }

        drivetrain.setControl(robot.driveFC
                    .withVelocityX(velocityX)
                    .withVelocityY(velocityY)
                    .withRotationalRate(-RobotContainer.driverRot * RobotContainer.MaxAngularRate)
                    .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
