// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class Intake extends Command {
  /** Creates a new RunIntakeCMD. */
  private IntakeSubsystem intakeSub;
  private LEDSubsystem ledSub;
  Timer startTime;
  Boolean finish;

  public Intake() {
    intakeSub = IntakeSubsystem.getInstance();
    ledSub = LEDSubsystem.getInstance();
    addRequirements(intakeSub, ledSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = new Timer();
    startTime.reset();
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(startTime.get());
    if(!intakeSub.getIntakeButton()) {
      startTime.start();
      intakeSub.stopIntake();
      // change LED light color
      ledSub.setGreen();
      intakeSub.rumbleDude();
    }
    else {
      intakeSub.runIntake(0.5);
    }
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.stopIntake();
    intakeSub.stopRumble();
    ledSub.setOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(startTime.get() > 0.5);
  }
}
