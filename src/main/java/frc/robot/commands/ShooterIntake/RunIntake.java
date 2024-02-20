// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class RunIntake extends Command {
 
  private Timer runtime;
  private IntakeSubsystem intakeSub;

  public RunIntake() {
    intakeSub = IntakeSubsystem.getInstance();
    runtime = new Timer();
    addRequirements(intakeSub);
  }

  @Override
  public void initialize() {
    runtime.restart();
  }

  @Override
  public void execute() {
    intakeSub.runIntake(0.25);

  }

  @Override
  public void end(boolean interrupted) {
    intakeSub.stopIntake();
    runtime.stop();
  }

  @Override
  public boolean isFinished() {
    // Stop after 3 seconds
    return runtime.get() > 1.5;
  }
}
