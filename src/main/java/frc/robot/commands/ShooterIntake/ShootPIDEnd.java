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

  /*
   * Trigger Command that fires when we see a massive drop in shooter RPM,
   * Meaning that there is a note going through the shooter
   */

  public ShootPIDEnd() {
    shootSub = ShooterSubsystem.getInstance();
    addRequirements(shootSub);
    overrideTimer = new Timer();
  }

  @Override
  public void initialize() {
    ogTarget = shootSub.getShooterVelocity();
    overrideTimer.reset();
    overrideTimer.start();
  }

  @Override
  public void execute() {
    currentRPM = shootSub.getShooterVelocity();
    // We may need to update ogTarget to be currentRPM. 
  }

  @Override
  public void end(boolean interrupted) {
    overrideTimer.stop();
    shootSub.runShooterPID(0);
  }

  @Override
  public boolean isFinished() {
    // Override Timer also fires so the robot can move on in auto even if it doesn't have a note. 
    return ((currentRPM <= ogTarget - 200) || overrideTimer.hasElapsed(1.5));
  }
}
