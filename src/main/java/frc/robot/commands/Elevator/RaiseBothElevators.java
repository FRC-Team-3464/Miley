// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class RaiseBothElevators extends Command {
  /** Creates a new RaiseBothElevators. */
  private ElevatorSubsystem elevatorSub;

  public RaiseBothElevators() {
    elevatorSub = ElevatorSubsystem.getInstance();
    addRequirements(elevatorSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSub.runRightElevator(0.05);
    elevatorSub.runLeftElevator(0.05);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSub.runRightElevator(0);
    elevatorSub.runLeftElevator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
