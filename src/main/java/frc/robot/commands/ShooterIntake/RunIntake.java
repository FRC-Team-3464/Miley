// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;


public class RunIntake extends Command {
 
  private Timer runtime;
  private IntakeSubsystem intakeSub;
  private LEDSubsystem ledSub;
  private double speedo;

  public RunIntake(double speed) {
    intakeSub = IntakeSubsystem.getInstance();
    ledSub = LEDSubsystem.getInstance();
    runtime = new Timer();
    addRequirements(intakeSub);
    speedo = speed;
  }

  @Override
  public void initialize() {
    runtime.restart();
  }

  @Override
  public void execute() {
    intakeSub.runIntake(speedo);
    ledSub.rainbow();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSub.stopIntakes();
    runtime.stop();
    ledSub.setOff();
  }

  @Override
  public boolean isFinished() {
    // Stop after 3 seconds
    // return runtime.get() > 1.5;
    return runtime.get() > 0.75;
  }
}
