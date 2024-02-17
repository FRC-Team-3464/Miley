// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAmp extends Command {
  /** Creates a new ShootAmp. */
  
  private ShooterSubsystem shootSub;
  public IntakeSubsystem intakeSub;

  public ShootAmp() {
    shootSub = ShooterSubsystem.getInstance();
    intakeSub = IntakeSubsystem.getInstance();
    addRequirements(shootSub);
    addRequirements(intakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // intakeSub.runIntake(0.3);
    // shootSub.runShooter(0.2);
    intakeSub.runIntake(0.05);
    shootSub.runShooter(0.05);

    shootSub.rumbleController(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.stopIntake();
    shootSub.stopShooter();
    shootSub.stopRumble();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
