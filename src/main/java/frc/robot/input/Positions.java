package frc.robot.input;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import frc.robot.drive.CommandSwerveDrivetrain;

public class Positions {
    
    private Positions() {
        throw new UnsupportedOperationException("Do not initialize");
    }

    public static double getDistanceToGoal(CommandSwerveDrivetrain drivetrain, Pose2d goalPose) {
        
        // get 2D translation from goal
        Pose2d robotPose = drivetrain.getState().Pose;
        Transform2d transformation = robotPose.minus(goalPose);

        // absolute distance (pythagorean)
        return Math.sqrt(
            Math.pow(transformation.getMeasureX().abs(Units.Meters), 2) + Math.pow(transformation.getMeasureY().abs(Units.Meters), 2)
        );
    }

}
