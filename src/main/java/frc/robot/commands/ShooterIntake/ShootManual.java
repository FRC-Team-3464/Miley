// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootManual extends Command {
  /** Creates a new ShootAmp. */
  
  private ShooterSubsystem shootSub;
  public IntakeSubsystem intakeSub;
  private LEDSubsystem ledSub;

  private Timer shootTimer;

  public ShootManual() {
    shootTimer = new Timer();
    shootSub = ShooterSubsystem.getInstance();
    intakeSub = IntakeSubsystem.getInstance();
    ledSub = LEDSubsystem.getInstance();
    addRequirements(shootSub);
    addRequirements(intakeSub);
    addRequirements(ledSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootTimer.reset();
    shootTimer.start();
    ledSub.setOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // intakeSub.runIntake(0.3);
    shootSub.runShooter(Constants.SandwichConstants.kSpeakerShootSpeed);
    if (shootTimer.get() > 1){
      intakeSub.runIntake(Constants.SandwichConstants.kIntakeSpeed);
    }

    shootSub.rumbleController(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootTimer.stop();
    shootTimer.reset();

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
