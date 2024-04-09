// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivoter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivoterSubsystem;

public class PivotAmpAndShoot extends Command {
  /** Creates a new PivotAmpAndShoot. */
  private final PivoterSubsystem pivoterSub;
  private final IntakeSubsystem intakeSub;

  public PivotAmpAndShoot() {
    pivoterSub = PivoterSubsystem.getInstance();
    intakeSub = IntakeSubsystem.getInstance();
    addRequirements(pivoterSub);
    addRequirements(intakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivoterSub.PIDPivot(29.7);
    if (Math.abs(29.7 - pivoterSub.getPivoterRawRotation()) <= 0.5) {
      intakeSub.runIntake(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
