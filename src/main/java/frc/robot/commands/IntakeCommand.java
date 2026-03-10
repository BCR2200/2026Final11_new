package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

    private IntakeSubsystem intakeSubsystem;
    private CommandSwerveDrivetrain drivetrain;
    private RobotContainer robotContainer;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, CommandSwerveDrivetrain drivetrain, RobotContainer robotContainer) {
        this.intakeSubsystem = intakeSubsystem;
        this.drivetrain = drivetrain;
        this.robotContainer = robotContainer;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intakeSubsystem, drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSubsystem.setIsIntaking(true);
        intakeSubsystem.setTiltPosition(IntakeSubsystem.tiltMaxExtensionPos);

        drivetrain.setControl(robotContainer.driveFCFA
            .withVelocityY(-RobotContainer.driverX * RobotContainer.MaxSpeed)
            .withVelocityX(-RobotContainer.driverY * RobotContainer.MaxSpeed)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withTargetDirection(new Rotation2d(-Math.atan2(RobotContainer.driverY, RobotContainer.driverX)))
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIsIntaking(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
