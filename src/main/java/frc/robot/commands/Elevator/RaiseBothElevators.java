// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class RaiseBothElevators extends Command {
  /** Creates a new RaiseBothElevators. */
  private ElevatorSubsystem elevatorSub;
  private LEDSubsystem ledSub;

  public RaiseBothElevators() {
    elevatorSub = ElevatorSubsystem.getInstance();
    ledSub = LEDSubsystem.getInstance();
    addRequirements(elevatorSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.SandwichConstants.noteMessage = "Pull up time! Raising arms!";
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSub.runRightElevator(Constants.ElevatorConstants.kElevatorRaiseSpeed);
    elevatorSub.runLeftElevator(Constants.ElevatorConstants.kElevatorRaiseSpeed);
    ledSub.bluePulse();
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSub.runRightElevator(0);
    elevatorSub.runLeftElevator(0);
    Constants.SandwichConstants.noteMessage = "Ready to pull!!";
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
