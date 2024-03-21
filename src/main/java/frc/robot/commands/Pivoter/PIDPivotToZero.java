// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivoter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivoterSubsystem;

public class PIDPivotToZero extends Command {
  /** Creates a new PIDPivotToZero. */

  private final PivoterSubsystem pivoterSub;
  private final double targetPosition;
  // Amount error that we can tolerate. 
  private final double PIVOTER_ANGLE_TOLERANCE = 0.75; // About 3 degrees
  double pivoterPositionError;
  public PIDPivotToZero(double target) {
    pivoterSub = PivoterSubsystem.getInstance();
    targetPosition = target;
    addRequirements(pivoterSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivoterSub.PIDPivot(-5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivoterSub.PIDPivot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivoterSub.getLowerSwitchToggled();
  }
}