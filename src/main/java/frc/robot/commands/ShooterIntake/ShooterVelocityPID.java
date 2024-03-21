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
  private LEDSubsystem ledSub;
  private double setPoint;
  private double error; 

  public ShooterVelocityPID(double target) {
    shootSub = ShooterSubsystem.getInstance();
    ledSub = LEDSubsystem.getInstance();
    addRequirements(shootSub);
    // addRequirements(ledSub);
    setPoint = target;
  }

  @Override
  public void initialize() {
    if(setPoint == 0) {
      ledSub.setOff();
    }
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
  public void end(boolean interrupted) {
    ledSub.setPurple();
  }


  @Override
  public boolean isFinished() {
    // End command once we're within tolerance. 
    return (error < 300);
  }
}
