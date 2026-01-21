package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.PIDMotor;

public class MTFTFFTHTTSSubsystem extends SubsystemBase {
    private boolean isFeeding = false;
    private double feedingSpeed; // it's a percent output from -1.0 to 1.0
    private boolean velocityMode = true;
    public PIDMotor feedPIDMotor;

    public MTFTFFTHTTSSubsystem() {
        // These numbers are placeholders, we don't actually know what they should be yet
        feedPIDMotor = PIDMotor.makeMotor(Constants.FEEDER_MOTOR_ID, 
                         "feeder", 1.0, 0.0, 0.1, 
                                        0.25, 0.1, 0.01, 
                                        100.0, 300.0, 0.00);
        feedPIDMotor.setCurrentLimit(30);
        feedPIDMotor.setIdleCoastMode();
    }

    public boolean getIsFeeding() { 
        return isFeeding; 
    }

    public void setIsFeeding(boolean feeding) {
        isFeeding = feeding;
    }

    public double getFeedingSpeed() {
        return feedingSpeed;
    }

    public void setFeedingSpeed(double speed) {
        feedingSpeed = speed;
    }

    // if you want to decrease the speed, use a negative increment
    public void incrementFeedingSpeed(double increment) {
        if (velocityMode) feedingSpeed += increment;
        else feedingSpeed = ExtraMath.clamp(feedingSpeed + increment, -1.0, 1.0);
    }

    @Override
    public void periodic() {
        if(velocityMode) {
            if (isFeeding)  feedPIDMotor.setVelocityTarget(feedingSpeed); 
            else            feedPIDMotor.setPercentOutput(0);
        }
        else {
            if (isFeeding)  feedPIDMotor.setPercentOutput(feedingSpeed);
            else            feedPIDMotor.setPercentOutput(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
