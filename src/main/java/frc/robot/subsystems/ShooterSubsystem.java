// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax shooterMotor = new CANSparkMax(11, MotorType.kBrushless);
  private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
  private final XboxController controller = new XboxController(2);

  private static ShooterSubsystem instance = null;

  public ShooterSubsystem() {
    shooterMotor.setInverted(false); 
    shooterMotor.setSmartCurrentLimit(30);
  }

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  // Note: Shooting must have the motor turning clockwise
  
  public void runShooter(double speed) {
    shooterMotor.set(speed);
  }

  public double getShooterEncoder() {
    return shooterEncoder.getPosition();
  }

  public void resetShooter() {
    shooterEncoder.setPosition(0);
  }

  public void stopShooter() {
    shooterMotor.set(0);
  }

  public void rumbleController(double rumbleStrength) {
    controller.setRumble(RumbleType.kBothRumble, rumbleStrength);
  }

  public void stopRumble() {
    controller.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
