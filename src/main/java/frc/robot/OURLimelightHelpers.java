package frc.robot;

import java.util.Arrays;

public class OURLimelightHelpers {

    public record LimelightContour(boolean hasTarget, double offsetX, double offsetY) {}

    public static double[] getJohnJawbreakerTaylorPercentages() {
        double[] dataFromLimelight = LimelightHelpers.getLimelightNTTableEntry("limelight", "llpython").getDoubleArray(new double[8]);
        return Arrays.copyOfRange(dataFromLimelight, 3, 6);
    }
    public static LimelightContour getContour() {
        // stored as int, we want bool
        boolean tv = LimelightHelpers.getLimelightNTTableEntry("limelight", "tv").getInteger(0) == 1;
        double tx = LimelightHelpers.getLimelightNTTableEntry("limelight", "txnc").getDouble(0);
        double ty = LimelightHelpers.getLimelightNTTableEntry("limelight", "tync").getDouble(0);
        return new LimelightContour(tv, tx, ty);
    }
}
