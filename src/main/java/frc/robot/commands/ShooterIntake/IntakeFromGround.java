
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SandwichConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class IntakeFromGround extends Command {
  /** Creates a new RunIntakeCMD. */
  private IntakeSubsystem intakeSub;
  private LEDSubsystem ledSub;
  Timer startTime;
  Boolean finish;
  Boolean note;

  public IntakeFromGround() {
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
    note = false;
    ledSub.setOrange();
    intakeSub.runServo(0);
    Constants.SandwichConstants.noteMessage = "Eating! nom nom nom";
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Constants.SandwichConstants.hasNote){
      startTime.start();
      ledSub.setGreen();
      intakeSub.rumbleDude();
      intakeSub.runExtendedIntake(0);
      intakeSub.runIntake(-0.05); // Backspit by 5% speed for 0.5 seconds. 
      note = true;
      Constants.SandwichConstants.noteMessage = "Note aquired! Time to upchuck!";
    }
    else{
      intakeSub.runIntake(SandwichConstants.kIntakeSpeed);
      intakeSub.runExtendedIntake(SandwichConstants.kExtendedIntakeSpeed);
    }
  }
  
    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    startTime.stop();
    startTime.reset();
    // ledSub.setGreen();
    intakeSub.stopRumble();

    if(note == false) {
      ledSub.setOff();
      Constants.SandwichConstants.noteMessage = "Womp womp. Try again. I'm still hungry :(";
    }

    intakeSub.runServo(0.5);
    intakeSub.stopIntakes();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return startTime.get() > 0.5;
  }
}
