// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootPIDEnd extends Command {
  /** Creates a new ShootPIDEnd. */
  private ShooterSubsystem shootSub;
  private double ogTarget;
  private double currentRPM;
  private final Timer overrideTimer;

  public ShootPIDEnd() {
    shootSub = ShooterSubsystem.getInstance();
    addRequirements(shootSub);
    overrideTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ogTarget = shootSub.getShooterVelocity();
    overrideTimer.reset();
    overrideTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentRPM = shootSub.getShooterVelocity();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootSub.runShooterPID(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((currentRPM <= ogTarget - 200) || overrideTimer.hasElapsed(1.5));
  }
}
