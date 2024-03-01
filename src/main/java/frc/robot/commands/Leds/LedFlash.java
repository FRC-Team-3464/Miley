// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LedFlash extends Command {
  /** Creates a new LedFlash. */
  private LEDSubsystem ledSub;
  Timer startTime;

  
  public LedFlash() {
    ledSub = LEDSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = new Timer();
    startTime.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(Math.floor(startTime.get()));
    if(Math.floor(startTime.get()) % 2 == 0){
      ledSub.setPurple();
    }else{
      ledSub.setOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledSub.setOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (startTime.get() >= 5);
  }
}
