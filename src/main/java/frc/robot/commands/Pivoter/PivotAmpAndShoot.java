// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivoter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PivoterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivoterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PivotAmpAndShoot extends Command {
  /** Creates a new PivotAmpAndShoot. */
  private final PivoterSubsystem pivoterSub;
  private final IntakeSubsystem intakeSub;
  private final ShooterSubsystem shootSub;

  public PivotAmpAndShoot() {
    pivoterSub = PivoterSubsystem.getInstance();
    intakeSub = IntakeSubsystem.getInstance();
    shootSub = ShooterSubsystem.getInstance();
    addRequirements(pivoterSub);
    addRequirements(intakeSub);
    addRequirements(shootSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(pivoterSub.getPivoterRawRotation() > 20) {
    pivoterSub.PIDPivot(PivoterConstants.kPostAmpPivoterRotations);
    if (Math.abs(PivoterConstants.kPostAmpPivoterRotations - pivoterSub.getPivoterRawRotation()) <= 0.5) {
      intakeSub.runIntake(Constants.SandwichConstants.kAmpShootSpeed);
      shootSub.runShooter(Constants.SandwichConstants.kAmpShootSpeed);
      Constants.SandwichConstants.noteMessage = "ssssspit. patooie";
    }
    }
    else {
      intakeSub.runIntake(Constants.SandwichConstants.kAmpShootSpeed);
      shootSub.runShooter(Constants.SandwichConstants.kAmpShootSpeed);
      Constants.SandwichConstants.noteMessage = "chomp~";
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootSub.stopShooter();
    intakeSub.stopIntakes();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
