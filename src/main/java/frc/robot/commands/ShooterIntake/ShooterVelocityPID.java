// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterVelocityPID extends Command {
  /** Creates a new ShootPID. */
  private ShooterSubsystem shootSub;
  private double setPoint;
  private double error; 

  public ShooterVelocityPID(double target) {
    shootSub = ShooterSubsystem.getInstance();
    addRequirements(shootSub);
    setPoint = target;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    shootSub.runShooterPID(setPoint);

    error = Math.abs(setPoint - shootSub.getVelocity());
    SmartDashboard.putNumber("Shoot Setpoint", setPoint);
    SmartDashboard.putNumber("Shoot Velocity", shootSub.getVelocity());
    SmartDashboard.putNumber("Shoot Error", error);

  }

  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    // End command once we're within tolerance. 
    return (error < 150);
  }
}
