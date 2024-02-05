// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax shooterMotor = new CANSparkMax(11, MotorType.kBrushless);
  private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
    
  private static ShooterSubsystem instance = null;

  public ShooterSubsystem() {
    shooterMotor.setInverted(false); 
  }

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

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
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
