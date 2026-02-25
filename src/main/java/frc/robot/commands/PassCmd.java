// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassCmd extends Command {

  ShooterSubsystem johnShooterSubsystem;
  ShooterSubsystem jawbreakerShooterSubsystem;
  ShooterSubsystem taylorShooterSubsystem;
  CommandSwerveDrivetrain driveSubsystem;

  public PassCmd(CommandSwerveDrivetrain driveSubsystem, ShooterSubsystem shooterSubsystem1,
      ShooterSubsystem shooterSubsystem2, ShooterSubsystem shooterSubsystem3) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem1, shooterSubsystem2, shooterSubsystem3, driveSubsystem);
    this.johnShooterSubsystem = shooterSubsystem1;
    this.jawbreakerShooterSubsystem = shooterSubsystem2;
    this.taylorShooterSubsystem = shooterSubsystem3;
    this.driveSubsystem = driveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Only allow passing when in the neutral zone
    double distance = driveSubsystem.getState().Pose.getX();

    if (distance > 4.625 && distance < 11.916) { // Neutral zone

      RobotContainer.passing = true;

      johnShooterSubsystem.setActuatorToPassPosition(distance);
      jawbreakerShooterSubsystem.setActuatorToPassPosition(distance);
      taylorShooterSubsystem.setActuatorToPassPosition(distance);

      johnShooterSubsystem.setIsShooting(true);
      jawbreakerShooterSubsystem.setIsShooting(true);
      taylorShooterSubsystem.setIsShooting(true);
      if (johnShooterSubsystem.isShooterAtSpeed()) {
        johnShooterSubsystem.setIsFeeding(true);
      }
      if (jawbreakerShooterSubsystem.isShooterAtSpeed()) {
        jawbreakerShooterSubsystem.setIsFeeding(true);
      }
      if (taylorShooterSubsystem.isShooterAtSpeed()) {
        taylorShooterSubsystem.setIsFeeding(true);
      }

    }
    else {
      RobotContainer.passing = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    johnShooterSubsystem.setIsShooting(false);
    johnShooterSubsystem.setIsFeeding(false);
    jawbreakerShooterSubsystem.setIsShooting(false);
    jawbreakerShooterSubsystem.setIsFeeding(false);
    taylorShooterSubsystem.setIsShooting(false);
    taylorShooterSubsystem.setIsFeeding(false);
    RobotContainer.passing = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
