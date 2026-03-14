package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ExtraMath;
import frc.robot.RobotContainer;

/**
 * Command that drives the robot along the outside of the field, hugging the nearest wall.
 *
 * This command finds the nearest wall and uses a P loop to maintain a set distance from it,
 * while allowing the driver to control movement along the wall and rotation.
 *
 * Field dimensions (2026 FRC): 16.54m long (X axis) x 8.05m wide (Y axis)
 */
public class BlendHugoCmd extends Command {

    private final RobotContainer robot;

    // Field dimensions in meters (2026 FRC field)
    private static final double FIELD_LENGTH = 16.54; // X axis
    private static final double FIELD_WIDTH = 8.05;   // Y axis

    // Wall positions
    private static final double LEFT_WALL_Y = 0.0;
    private static final double RIGHT_WALL_Y = FIELD_WIDTH;
    private static final double NEAR_WALL_X = 0.0;
    private static final double FAR_WALL_X = FIELD_LENGTH;

    // Target distance from wall (robot center to wall)
    private static final double WALL_OFFSET = 0.5; // meters

    // P gain for driving towards the wall
    private static final double kP = 2.0;

    // Maximum correction velocity (m/s)
    private static final double MAX_CORRECTION_VELOCITY = 3.0;

    private enum NearestWall {
        LEFT,   // Y = 0
        RIGHT,  // Y = FIELD_WIDTH
        NEAR,   // X = 0
        FAR     // X = FIELD_LENGTH
    }

    public BlendHugoCmd(RobotContainer robot) {
        addRequirements(robot.drivetrain);
        this.robot = robot;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pose2d currentPose = robot.drivetrain.getState().Pose;
        double x = currentPose.getX();
        double y = currentPose.getY();

        // Calculate perpendicular distance to each wall
        double distToLeftWall = y - LEFT_WALL_Y;
        double distToRightWall = RIGHT_WALL_Y - y;
        double distToNearWall = x - NEAR_WALL_X;
        double distToFarWall = FAR_WALL_X - x;

        // Find the nearest wall
        double minDist = Math.min(Math.min(distToLeftWall, distToRightWall),
                                   Math.min(distToNearWall, distToFarWall));

        NearestWall nearestWall;
        if (minDist == distToLeftWall) {
            nearestWall = NearestWall.LEFT;
        } else if (minDist == distToRightWall) {
            nearestWall = NearestWall.RIGHT;
        } else if (minDist == distToNearWall) {
            nearestWall = NearestWall.NEAR;
        } else {
            nearestWall = NearestWall.FAR;
        }

        SmartDashboard.putString("WallHug/NearestWall", nearestWall.toString());
        SmartDashboard.putNumber("WallHug/DistanceToWall", minDist);

        // Calculate target position and error based on nearest wall
        double velocityX;
        double velocityY;

        switch (nearestWall) {
            case LEFT:
                // Target Y = WALL_OFFSET (just inside left wall)
                // Error is positive when robot needs to move towards lower Y
                double errorLeft = y - WALL_OFFSET;
                double correctionLeft = -kP * errorLeft; // Negative because we want to decrease Y
                correctionLeft = ExtraMath.clamp(correctionLeft, -MAX_CORRECTION_VELOCITY, MAX_CORRECTION_VELOCITY);

                // Driver controls X, P loop controls Y
                velocityX = -RobotContainer.driverY * RobotContainer.MaxSpeed;
                velocityY = correctionLeft;
                SmartDashboard.putNumber("WallHug/Error", errorLeft);
                break;

            case RIGHT:
                // Target Y = FIELD_WIDTH - WALL_OFFSET (just inside right wall)
                double errorRight = (FIELD_WIDTH - WALL_OFFSET) - y;
                double correctionRight = kP * errorRight; // Positive because we want to increase Y
                correctionRight = ExtraMath.clamp(correctionRight, -MAX_CORRECTION_VELOCITY, MAX_CORRECTION_VELOCITY);

                // Driver controls X, P loop controls Y
                velocityX = -RobotContainer.driverY * RobotContainer.MaxSpeed;
                velocityY = correctionRight;
                SmartDashboard.putNumber("WallHug/Error", errorRight);
                break;

            case NEAR:
                // Target X = WALL_OFFSET (just inside near wall)
                double errorNear = x - WALL_OFFSET;
                double correctionNear = -kP * errorNear; // Negative because we want to decrease X
                correctionNear = ExtraMath.clamp(correctionNear, -MAX_CORRECTION_VELOCITY, MAX_CORRECTION_VELOCITY);

                // P loop controls X, driver controls Y
                velocityX = correctionNear;
                velocityY = -RobotContainer.driverX * RobotContainer.MaxSpeed;
                SmartDashboard.putNumber("WallHug/Error", errorNear);
                break;

            case FAR:
            default:
                // Target X = FIELD_LENGTH - WALL_OFFSET (just inside far wall)
                double errorFar = (FIELD_LENGTH - WALL_OFFSET) - x;
                double correctionFar = kP * errorFar; // Positive because we want to increase X
                correctionFar = ExtraMath.clamp(correctionFar, -MAX_CORRECTION_VELOCITY, MAX_CORRECTION_VELOCITY);

                // P loop controls X, driver controls Y
                velocityX = correctionFar;
                velocityY = -RobotContainer.driverX * RobotContainer.MaxSpeed;
                SmartDashboard.putNumber("WallHug/Error", errorFar);
                break;
        }

        SmartDashboard.putNumber("WallHug/VelocityX", velocityX);
        SmartDashboard.putNumber("WallHug/VelocityY", velocityY);

        // Apply the control with driver rotation always available
        robot.drivetrain.setControl(robot.driveFC
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(-RobotContainer.driverRot * RobotContainer.MaxAngularRate)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));
    }

    @Override
    public void end(boolean interrupted) {
        robot.drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
