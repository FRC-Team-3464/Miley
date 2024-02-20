// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootPID extends Command {
  /** Creates a new ShootPID. */
  private ShooterSubsystem shootSub;
  // private IntakeSubsystem intakeSub;
  double setPoint;
  double error; 
  // Timer timeIt;

  // Boolean fishnish;

  public ShootPID(double target) {
    shootSub = ShooterSubsystem.getInstance();
    // intakeSub = IntakeSubsystem.getInstance();
    addRequirements(shootSub);
    // addRequirements(intakeSub);

    // timeIt = new Timer();
    setPoint = target;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // shootSub.startPIDthings();
    // timeIt.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shootSub.runShooterPID(setPoint);

    error = Math.abs(setPoint - shootSub.getVelocity());
    SmartDashboard.putNumber("Shoot Setpoint", setPoint);
    SmartDashboard.putNumber("Shoot Velocity", shootSub.getVelocity());
    SmartDashboard.putNumber("Shoot Error", error);


    // if(Math.abs(setPoint - shootSub.getVelocity()) < 100) {
    //   intakeSub.runIntake(0.25);
    //   timeIt.start();
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shootSub.runShooterPID(0);
    // intakeSub.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return timeIt.get() > 2.7;
    return (error < 50);
  }
}
