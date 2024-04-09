// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax intakeMotor = new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax miniMotor = new CANSparkMax(15, MotorType.kBrushless);
  private final CANSparkMax intakeMotorFollower = new CANSparkMax(16, MotorType.kBrushless);

  private final DigitalInput intakeButton = new DigitalInput(6);
  private final DigitalInput rightPhotoElectric = new DigitalInput(7);
  private final DigitalInput leftPhotoElectric = new DigitalInput(8);

  private final Servo servoStop = new Servo(1);

  private final XboxController xbox = new XboxController(2);

  private static IntakeSubsystem instance = null;  

  public IntakeSubsystem() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(true);
    intakeMotorFollower.restoreFactoryDefaults();
    intakeMotorFollower.follow(intakeMotor, true);
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

  public void runServo(double position) {
    servoStop.set(position);
  }

  public Boolean getPhotoElectricLeft() {
    return leftPhotoElectric.get();
  }

  public Boolean getPhotoElectricRight() {
    return rightPhotoElectric.get();
  }
  
  public double getIntakeVelocity(){
    return intakeMotor.get();
  }

  public void rumbleDude() {
    xbox.setRumble(RumbleType.kBothRumble, 0.5);
  }

  public void stopRumble() {
    xbox.setRumble(RumbleType.kBothRumble, 0);
  }

  public double getCurrent(){
    return intakeMotor.getOutputCurrent();
  }
  
  public void runExtendedIntake(double speed) {
    miniMotor.set(speed);
  }

  public void runIntake(double speed) {
    intakeMotor.setInverted(true);
    intakeMotorFollower.follow(intakeMotor, true);

    intakeMotor.set(speed);
  }

  public void stopIntakes() {
    intakeMotor.set(0);
    // intakeMotorFollower.set(0);
    miniMotor.set(0);
  }
// hello this is also a test yo0iehnlkan

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("Note? - 6", intakeButton.get());
    SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake Follower Current", intakeMotorFollower.getOutputCurrent());

    
    SmartDashboard.putBoolean("Right Photoelectric - 7", getPhotoElectricRight());
    SmartDashboard.putBoolean("Left Photoelectric - 8", getPhotoElectricLeft());
    SmartDashboard.putBoolean("ANY Photoelectric?", getPhotoElectricLeft() || getPhotoElectricRight());
    

    
    // This method will be called once per scheduler run
  }
}
