// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final DigitalInput elevatorLMin = new DigitalInput(0);
  private final DigitalInput elevatorLMax = new DigitalInput(1);
  private final DigitalInput elevatorRMin = new DigitalInput(2);
  private final DigitalInput elevatorRMax = new DigitalInput(3);

  private final CANSparkMax leftElevatorMotor = new CANSparkMax(9, MotorType.kBrushless);
  private final CANSparkMax rightElevatorMotor = new CANSparkMax(10, MotorType.kBrushless);

  Boolean leftResistance;
  Boolean rightResistance;

  public ElevatorSubsystem() {
    leftResistance = false;
    rightResistance = false;
    leftElevatorMotor.setSmartCurrentLimit(30);
    rightElevatorMotor.setSmartCurrentLimit(30);
    leftElevatorMotor.setInverted(true);
    rightElevatorMotor.setInverted(false);
  }

  private static ElevatorSubsystem instance = null;

  public static ElevatorSubsystem getInstance() {
    if (instance == null) {
      instance = new ElevatorSubsystem();
    }
    return instance;
  }

  public Boolean getLeftRes() {
    return leftResistance;
  }

  public Boolean getRightRes() {
    return rightResistance;
  }

  public double getLeftOutput() {
    return leftElevatorMotor.getAppliedOutput();
  }

  public double getRightOutput() {
    return rightElevatorMotor.getAppliedOutput();
  }

  public void setLeftRes(Boolean yapOne) {
    leftResistance = yapOne;
  }

  public void setRightRes(Boolean yapTwo) {
    rightResistance = yapTwo;
  }


  public void runLeftElevator(double speed) {
    if(speed < 0) {
      if(!elevatorLMin.get()) {
        leftElevatorMotor.set(0);
      }
      else {
        leftElevatorMotor.set(speed);
      }
    }
    else if(speed > 0) {
      if(!elevatorLMax.get()) {
        leftElevatorMotor.set(0);
      }
      else {
        leftElevatorMotor.set(speed);
      }
    }
    else {
      leftElevatorMotor.set(0);
    }

  }

  public void runRightElevator(double speed) {
    if(speed < 0) {
      if(!elevatorRMin.get()) {
        rightElevatorMotor.set(0);
      }
      else {
        rightElevatorMotor.set(speed);
      }
    }
    else if(speed > 0) {
      if(!elevatorRMax.get()) {
        rightElevatorMotor.set(0);
      }
      else {
        rightElevatorMotor.set(speed);
      }
    }
    else {
      rightElevatorMotor.set(0);
    }
  }

  public Boolean leftElevatorMax() {
    return elevatorLMax.get();
  }

  public Boolean leftElevatorMin() {
    return elevatorLMin.get();
  }

  public Boolean rightElevatorMax() {
    return elevatorRMax.get();
  }

  public Boolean rightElevatorMin() {
    return elevatorRMin.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Left Elevator Max - 1", elevatorLMax.get());
    SmartDashboard.putBoolean("Left Elevator Min - 0", elevatorLMin.get());
    SmartDashboard.putBoolean("Right Elevator Max - 3", elevatorRMax.get());
    SmartDashboard.putBoolean("Right Elevator Min - 2", elevatorRMin.get());
  }
}
