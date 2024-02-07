// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoLowerElevators extends Command {
  /** Creates a new ExtendLeftElevator. */
  private ElevatorSubsystem elevatorSub;

  public AutoLowerElevators() {
    elevatorSub = ElevatorSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSub);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSub.setLeftRes(false);
    elevatorSub.setRightRes(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(elevatorSub.getLeftOutput() > 10) {
      elevatorSub.setLeftRes(true);
    }

    if(elevatorSub.getRightOutput() > 10) {
      elevatorSub.setRightRes(true);
    }

    if(!elevatorSub.getLeftRes()) {
      elevatorSub.runLeftElevator(-0.5);
    }

    if(!elevatorSub.getRightRes()) {
      elevatorSub.runRightElevator(-0.5);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSub.runLeftElevator(0);
    elevatorSub.runRightElevator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
