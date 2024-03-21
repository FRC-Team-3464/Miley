
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(!intakeSub.getIntakeButton()) {
    //   buttonTime.start();
    //   // change LED light color
    //   ledSub.setGreen();
    //   intakeSub.rumbleDude();
    //   note = true;
    // }
    if(intakeSub.getPhotoElectricRight() || intakeSub.getPhotoElectricLeft()){
      startTime.start();
      ledSub.setGreen();
      note = true;
    }

    if(note){
      // intakeSub.runIntake(0.1);
      intakeSub.rumbleDude();
      intakeSub.runExtendedIntake(0);
      intakeSub.runIntake(-0);
    }
    else{
      intakeSub.runIntake(Constants.SandwichConstants.kIntakeSpeed);
      intakeSub.runExtendedIntake(0.85);
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
    }
    intakeSub.runServo(0.5);
    intakeSub.stopIntake();


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return startTime.get() > 0.5;
  }
}
