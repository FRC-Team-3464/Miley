// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAmp extends Command {
  /** Creates a new ShootAmp. */
  
  public final ShooterSubsystem shootSub;
  public final IntakeSubsystem intakeSub;

  public ShootAmp(ShooterSubsystem shootSub, IntakeSubsystem intakeSub) {
    this.shootSub = shootSub;
    this.intakeSub = intakeSub;
    addRequirements(shootSub);
    addRequirements(intakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.runIntake(0.3);
    shootSub.runShooter(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.stopIntake();
    shootSub.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
