package frc.robot.commands;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;

public class BlendAdamModeCmd extends Command {
    private static enum BlenderWalls {
        BLUE_HUB, WALL_LEFT_FROM_BLUE, RED_HUB, WALL_RIGHT_FROM_BLUE
    };

    private final RobotContainer robot;

    private CommandSwerveDrivetrain drivetrain;
    private Pose2d currentPose2d;

    private double MaxSpeed = RobotContainer.MaxSpeed;
    private double MaxAngularRate = RobotContainer.MaxAngularRate;

    private final SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private CommandXboxController m_driverController;

    //in meters
    private double blueHUBXLine = 5.7;

    //TODO FIND VALUE
    private double redHUBXLine = 11.6;

    private double wallLFromBYLine = 7.590;
    private double wallRFromBYLine = 0.319;



    public BlendAdamModeCmd(RobotContainer robot) {
        this.drivetrain = robot.drivetrain;
        this.m_driverController = robot.driverController;
        this.robot = robot;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (true) {
            currentPose2d = drivetrain.getState().Pose;
            var x = currentPose2d.getX();
            var y = currentPose2d.getY();

            /*
             * If (in a corner, allow driver X and Y)
             * If (On one of the X lines, only allow Y)
             * If (On one of the Y lines, only allow X)
             * If (Not on either, free reign)
             */

            if((ExtraMath.within(y, redHUBXLine, 0.5)
                || ExtraMath.within(y, blueHUBXLine, 0.5)) &&
                    (ExtraMath.within(x, wallLFromBYLine, 0.5)
                || ExtraMath.within(x, wallRFromBYLine, 0.5))){

                    drivetrain.setControl(driveFC.withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed));

                    SmartDashboard.putString("BlendState", "On Corner");
            }
            else if(ExtraMath.within(x, redHUBXLine, 0.5)
                || ExtraMath.within(x, blueHUBXLine, 0.5)){
                    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());

                    double robotVelocityX = fieldSpeeds.vxMetersPerSecond;
                    double robotVelocityY = fieldSpeeds.vyMetersPerSecond;

                    if(ExtraMath.within(x, redHUBXLine, 0.5)){
                        SmartDashboard.putString("BlendState", "On Red Hub Line");
                        //TODO Is > and angle correct?
                        if(robotVelocityY > 0){
                            robot.drivetrain.setControl(robot.driveFCFA.withTargetDirection(Rotation2d.fromDegrees(0)).withVelocityY(-m_driverController.getLeftX() * MaxSpeed));
                        } else {
                            robot.drivetrain.setControl(robot.driveFCFA.withTargetDirection(Rotation2d.fromDegrees(180)));
                        }
                    } else if (ExtraMath.within(x, blueHUBXLine, 0.5)){
                        SmartDashboard.putString("BlendState", "On Blue Hub Line");
                        if(robotVelocityY > 0){
                            robot.drivetrain.setControl(robot.driveFCFA.withTargetDirection(Rotation2d.fromDegrees(180)).withVelocityY(-m_driverController.getLeftX() * MaxSpeed));
                        } else {
                            robot.drivetrain.setControl(robot.driveFCFA.withTargetDirection(Rotation2d.fromDegrees(0)));
                        }
                    }

            }
            else if(ExtraMath.within(y, wallLFromBYLine, 0.5)
                || ExtraMath.within(y, wallRFromBYLine, 0.5)){
                ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());

                double robotVelocityX = fieldSpeeds.vxMetersPerSecond;
                double robotVelocityY = fieldSpeeds.vyMetersPerSecond;

                if(ExtraMath.within(y, wallRFromBYLine, 0.5)){
                    SmartDashboard.putString("BlendState", "On Right From Blue Line");
                    //TODO Is > and angle correct?
                    if(robotVelocityX > 0){
                        robot.drivetrain.setControl(robot.driveFCFA.withTargetDirection(Rotation2d.fromDegrees(90)).withVelocityY(-m_driverController.getLeftX() * MaxSpeed));
                    } else {
                        robot.drivetrain.setControl(robot.driveFCFA.withTargetDirection(Rotation2d.fromDegrees(270)));
                    }
                } else if (ExtraMath.within(y, wallLFromBYLine, 0.5)){
                    SmartDashboard.putString("BlendState", "On Left From Blue Line");
                    if(robotVelocityX > 0){
                        robot.drivetrain.setControl(robot.driveFCFA.withTargetDirection(Rotation2d.fromDegrees(270)).withVelocityY(-m_driverController.getLeftX() * MaxSpeed));
                    } else {
                        robot.drivetrain.setControl(robot.driveFCFA.withTargetDirection(Rotation2d.fromDegrees(90)));
                    }
                }
            } else {
                SmartDashboard.putString("BlendState", "Free Roam");
                drivetrain.setControl(driveFC.withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed));
            } 
        }
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
