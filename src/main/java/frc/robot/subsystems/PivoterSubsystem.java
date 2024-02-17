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

  private final CANSparkMax leftPivoter = new CANSparkMax(13, MotorType.kBrushless);
  private final CANSparkMax rightPivoter = new CANSparkMax(14, MotorType.kBrushless);

  private final DigitalInput leftLimit = new DigitalInput(4);
  private final DigitalInput rightLimit = new DigitalInput(5);

  private final RelativeEncoder leftEncoder = leftPivoter.getEncoder();
  private static PivoterSubsystem instance = null;  


  public PivoterSubsystem() {
    leftPivoter.setInverted(false);
    rightPivoter.follow(leftPivoter, true);
  }

  public static PivoterSubsystem getInstance() {
    if (instance == null) {
      instance =  new PivoterSubsystem();
    }
    return instance;
  }

  public void pivot(double speed) {
    if(speed < 0 && getSwitchToggled()){
      // If we hit the switch and we're going down, stop the motor and reset the encoder
      resetEncoder();
      leftPivoter.set(0);
    }else if (speed > 0 && (getPivoterDegrees() > PivoterConstants.kMaxPivoterDegrees)) {
      // If we're going up and exceed our degrees for the pivoter, stop. 
      leftPivoter.set(0);

    } else {
      leftPivoter.set(speed);
    }
  }

  public void stopMotor() {
    leftPivoter.stopMotor();
  }

  public double getPivoterSpeed() {
    // Get the speed of the motor. 
    return leftPivoter.get();
  }

  public double getPivoterRawRotation(){
    // Return the pivoter raw position in rotations. 
    return leftEncoder.getPosition();
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
    leftEncoder.setPosition(0);
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
    if(getSwitchToggled()){
      resetEncoder();
    };

    // Print debug information
    SmartDashboard.putBoolean("4 - Pivotor Left Switch", getLeftSwitch());
    SmartDashboard.putBoolean("5 - Pivotor Right Switch", getRightSwitch());
    SmartDashboard.putBoolean("Pivotor Switch Input", getSwitchToggled());


    SmartDashboard.putNumber("Pivotor Raw Rotations", getPivoterRawRotation());
    SmartDashboard.putNumber("Pivotor Rotations", getPivoterRotation());
    SmartDashboard.putNumber("Pivotor Degrees", getPivoterDegrees());

    SmartDashboard.putNumber("Pivotor Current", leftPivoter.getOutputCurrent());
  }
}
