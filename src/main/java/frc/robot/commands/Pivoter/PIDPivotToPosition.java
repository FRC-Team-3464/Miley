// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivoter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivoterConstants;
import frc.robot.subsystems.PivoterSubsystem;

public class PIDPivotToPosition extends Command {
  private final PivoterSubsystem pivoterSub;
  private final double targetPosition;
  // Amount error that we can tolerate. 
  private final double PIVOTER_ANGLE_TOLERANCE = 0.75; // About 3 degrees
  double pivoterPositionError;

  public PIDPivotToPosition(double target) {
    pivoterSub = PivoterSubsystem.getInstance();
    targetPosition = target;

    addRequirements(pivoterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivoterSub.PIDPivot(targetPosition); // setpoint is rotations.
    
    double currPositionRotations = pivoterSub.getPivoterRawRotation();

    pivoterPositionError = Math.abs(targetPosition - currPositionRotations);
    SmartDashboard.putNumber("Pivoter Target Rotation", targetPosition);
    SmartDashboard.putNumber("Pivoter Reading", currPositionRotations);
    SmartDashboard.putNumber("Pivoter Error", pivoterPositionError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Make sure we are not higher than what need
    // ... Don't this really matters - the setpoint is going to be set either way...
    if(pivoterSub.getPivoterRotation() >= PivoterConstants.kMaxPivoterRotations){
      return true;
    }
    // End the command if we are within our PIVOTER_ANGLE_TOLERANCE
    return (pivoterPositionError < PIVOTER_ANGLE_TOLERANCE);
  }
}
