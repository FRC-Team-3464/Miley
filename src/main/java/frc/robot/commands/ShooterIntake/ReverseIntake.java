// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ReverseIntake extends Command {
  /** Creates a new ReverseIntakeCMD. */
  private IntakeSubsystem intakeSub;
  private ShooterSubsystem shooterSub;
  
  public ReverseIntake() {
    intakeSub = IntakeSubsystem.getInstance();
    shooterSub = ShooterSubsystem.getInstance();
    addRequirements(intakeSub);
    addRequirements(shooterSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.SandwichConstants.noteMessage = "ick";
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.runIntake(Constants.SandwichConstants.kReverseIntakeSpeed);
    shooterSub.runShooter(Constants.SandwichConstants.kReverseShootSpeed);
    intakeSub.runExtendedIntake(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.stopIntakes();
    shooterSub.stopShooter();
  }

  //hello this is a test oijlknav
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
