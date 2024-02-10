// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivoterSubsystem extends SubsystemBase {
  /** Creates a new PivoterSubsystem. */
  private final CANSparkMax pivotMotor = new CANSparkMax(13, MotorType.kBrushless);
  private final CANSparkMax secondPivotMotor = new CANSparkMax(14, MotorType.kBrushless);

  private final DigitalInput leftLimit = new DigitalInput(4);
  private final DigitalInput rightLimit = new DigitalInput(5);

  private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
  private static PivoterSubsystem instance = null;  


  public PivoterSubsystem() {
    pivotMotor.setInverted(false);
    secondPivotMotor.follow(pivotMotor, true);
  }

  public static PivoterSubsystem getInstance() {
    if (instance == null) {
      instance =  new PivoterSubsystem();
    }
    return instance;
  }

  public void pivot(double speed) {
    pivotMotor.set(speed);
  }

  public void stopMotor() {
    pivotMotor.stopMotor();
  }

  public double getPivoterSpeed() {
    // Get the speed of the motor. 
    return pivotMotor.get();
  }

  public double getPivoterRotation(){
    // Return the pivoter position in rotations. 
    return pivotEncoder.getPosition();
  }

  public double getPivoterDegrees(){
    // MUST FIND THE ROTATION TO DEGREES FACTOR!!!!!
    // GO TO MANCHESTER!!!
    // DON'T FORGET THIS!!!
    // PLEASE SOMEBODY READ THIS AND DON'T FORGET!!!!
    return pivotEncoder.getPosition(); 
  }

  public void resetEncoder(){
    // Set the encoder back to normal
    pivotEncoder.setPosition(0);
  }

  public Boolean getLeftSwitch() {
    return !leftLimit.get();
  }

  public Boolean getRightSwitch() {
    return !rightLimit.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
