// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSpeaker extends Command {
  /** Creates a new RunShooterCMD. */
  
  // Made private following guide 
  // https://docs.google.com/document/d/1rMpvW10_W3HbNHDALNwipZNWk1XB1qKxHVw3UQViRO0/edit
  private ShooterSubsystem shootSub;
  private IntakeSubsystem intakeSub;

  public ShootSpeaker() {
    shootSub = ShooterSubsystem.getInstance();
    intakeSub = IntakeSubsystem.getInstance();
    addRequirements(shootSub);
    addRequirements(intakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootSub.resetShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shootSub.runShooter(1);
    if(shootSub.getShooterEncoder() >= 20) {
      intakeSub.runIntake(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootSub.stopShooter();
    shootSub.resetShooter();
    intakeSub.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shootSub.getShooterEncoder() > 50;
  }
}
