// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivoterSubsystem;

public class PivotToPosition extends Command {
  /** Creates a new PivotToAmp. */
  private PivoterSubsystem pivotSub;
  private final double setpoint;
  private double startingPos;

  public PivotToPosition(double target) {
    pivotSub = PivoterSubsystem.getInstance();
    addRequirements(pivotSub);
    setpoint = target;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingPos = pivotSub.getPivoterDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(startingPos < setpoint) {
      pivotSub.pivot(0.5);
    }
    else {
      pivotSub.pivot(-0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSub.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(pivotSub.getPivoterDegrees() >= 150 /*MUST CHANGE THIS TO A MAXIMUM DEGREES */) {
      return true;
    }
    else if(!pivotSub.getLeftSwitch() || !pivotSub.getRightSwitch()) {
      return true;
    }
    return (Math.abs(pivotSub.getPivoterDegrees() - setpoint) < 1);
  }
}
