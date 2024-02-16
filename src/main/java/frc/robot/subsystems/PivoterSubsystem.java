// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivoterConstants;

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
    // If we hit the switch and we're going down, stop
    if(speed < 0 && getSwitchToggled()){
      pivotMotor.set(0);
    }else{
      pivotMotor.set(speed);
    }
  }

  public void stopMotor() {
    pivotMotor.stopMotor();
  }

  public double getPivoterSpeed() {
    // Get the speed of the motor. 
    return pivotMotor.get();
  }

  public double getPivoterRawRotation(){
    // Return the pivoter raw position in rotations. 
    return pivotEncoder.getPosition();
  }

  public double getPivoterRotation(){
    // Return the pivoter raw position in rotations. 
    return getPivoterRawRotation() * PivoterConstants.kPivoterGearRatio;
  }

  public double getPivoterDegrees(){
    // 360 degrees in a rotation
    return getPivoterRotation() * 360; 
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

  public Boolean getSwitchToggled(){
    //Return if any of the switches were toggled
    return (getLeftSwitch() || getRightSwitch());
  }

  @Override
  public void periodic() {
    // Print debug information
    SmartDashboard.putBoolean("Pivotor Left Switch", getLeftSwitch());
    SmartDashboard.putBoolean("Pivotor Right Switch", getRightSwitch());
    SmartDashboard.putBoolean("Pivotor Switch Input", getSwitchToggled());


    SmartDashboard.putNumber("Pivotor Raw Rotations", getPivoterRawRotation());
    SmartDashboard.putNumber("Pivotor Rotations", getPivoterRotation());
    SmartDashboard.putNumber("Pivotor Degrees", getPivoterDegrees());

    SmartDashboard.putNumber("Pivotor Current", pivotMotor.getOutputCurrent());
  }
}
