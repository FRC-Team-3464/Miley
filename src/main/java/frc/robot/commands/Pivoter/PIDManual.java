// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivoter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivoterConstants;
import frc.robot.subsystems.PivoterSubsystem;

public class PIDManual extends Command {
  /** Creates a new PIDManual. */
  private boolean isUp;
  private PivoterSubsystem pivoterSub;
  private double currentPivoterReading;
  private double target;
  private final double PIVOTER_ANGLE_TOLERANCE = 0.75;
  double pivoterPositionError;

  public PIDManual(boolean isUp) {
    pivoterSub = PivoterSubsystem.getInstance();
    this.isUp = isUp;

    addRequirements(pivoterSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPivoterReading = pivoterSub.getPivoterRawRotation();
    if (isUp){
      target = currentPivoterReading + 0.5;
    }else{
      target = currentPivoterReading - 1;
    }
    SmartDashboard.putNumber("Manual Pivoter Target Rotation", target);

    // if ((target >= PivoterConstants.kMaxPivoterRotations) || target <= 0){
    //   System.out.println("ENDING");
    //   // end command automatically if we're done. 
    //   end(false);
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currPositionRotations = pivoterSub.getPivoterRawRotation();
    pivoterPositionError = Math.abs(target - currPositionRotations);

    // SmartDashboard.putNumber("Pivoter Target Rotation", Units.rotationsToDegrees(targetPosition));
    SmartDashboard.putNumber("Manual Pivoter Reading", currPositionRotations);
    SmartDashboard.putNumber("Manual Pivoter Error", pivoterPositionError);
 
    pivoterSub.PIDPivot(target); // setpoint is rotations.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(pivoterSub.getPivoterRawRotation() >= PivoterConstants.kMaxPivoterRotations){
      return true;
    }
    return (pivoterPositionError < PIVOTER_ANGLE_TOLERANCE);
  }
}
