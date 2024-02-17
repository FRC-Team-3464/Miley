// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax intakeMotor = new CANSparkMax(12, MotorType.kBrushless);
  private final DigitalInput intakeButton = new DigitalInput(6);
  private final XboxController xbox = new XboxController(2);

  private static IntakeSubsystem instance = null;  

  public IntakeSubsystem() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
  }

  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }  
    return instance;
  }
  
  public Boolean getIntakeButton() {
    return intakeButton.get();
  }

  public void rumbleDude() {
    xbox.setRumble(RumbleType.kBothRumble, 0.5);
  }

  public void stopRumble() {
    xbox.setRumble(RumbleType.kBothRumble, 0);
  }

  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }
// hello this is also a test yo0iehnlkan

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note? - 6", intakeButton.get());
    // This method will be called once per scheduler run
  }
}
