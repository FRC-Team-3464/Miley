// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivoterConstants;

public class PivoterSubsystem extends SubsystemBase {
  /** Creates a new PivoterSubsystem. */

  private final CANSparkMax leftPivoter = new CANSparkMax(13, MotorType.kBrushless);
  private final CANSparkMax rightPivoter = new CANSparkMax(14, MotorType.kBrushless);
  // private final CANSparkMax leftPivoter = new CANSparkMax(14, MotorType.kBrushless);
  // private final CANSparkMax rightPivoter = new CANSparkMax(13, MotorType.kBrushless);


  private final DigitalInput leftLimit = new DigitalInput(5); // Min value
  private final DigitalInput rightLimit = new DigitalInput(4); // Max value

  private final RelativeEncoder leftEncoder = leftPivoter.getEncoder();
  private final RelativeEncoder rightEncoder = rightPivoter.getEncoder();

  private static PivoterSubsystem instance = null;  

  private final SparkPIDController m_pidController;
  // PID gains for Spark Smart Motion - class defined below
  // private Gains gains = new Gains(0.00025, 0, 0, 0.00025, 0, 1);
  private Gains gains = new Gains(0.0001, 0, 0, 0.0004, 0, 1);

  private static final int SMART_MOTION_SLOT = 0;

    // SmartMotion configs
    // private static final double MAX_VELOCITY_RPM = 1_000; // NEO free speed 5676 RPM
    // private static final double MIN_VELOCITY_RPM = 0;
    // private static final double MAX_ACCELERATION_RPM_PER_SEC = 1_500;
    // private static final double ALLOWED_ERROR = 0.1; //motor rotations
    private static final double MAX_VELOCITY_RPM = 1000; // NEO free speed 5676 RPM
    private static final double MIN_VELOCITY_RPM = 0;
    private static final double MAX_ACCELERATION_RPM_PER_SEC = 1500;
    private static final double ALLOWED_ERROR = 0.075; //motor rota

    // Voltage needed to maintain horizontal arm position.
    private static final double horizontalArbFF = 0.00; 

  public PivoterSubsystem() {
    leftPivoter.restoreFactoryDefaults();
    rightPivoter.restoreFactoryDefaults();
    
    leftPivoter.setInverted(true);
    rightPivoter.follow(leftPivoter, true);

    // Define PIDController to be the one in the SparkMax
    m_pidController = leftPivoter.getPIDController();
    // m_pidController = leftPivoter.getPIDController();


    // Set PID coefficients
    m_pidController.setP(gains.kP, SMART_MOTION_SLOT);
    m_pidController.setI(gains.kI, SMART_MOTION_SLOT);
    m_pidController.setD(gains.kD, SMART_MOTION_SLOT);
    m_pidController.setIZone(gains.kIzone, SMART_MOTION_SLOT);
    m_pidController.setFF(gains.kF, SMART_MOTION_SLOT);
    m_pidController.setOutputRange(-gains.kPeakOutput, gains.kPeakOutput, SMART_MOTION_SLOT);

    m_pidController.setSmartMotionMaxVelocity(MAX_VELOCITY_RPM, SMART_MOTION_SLOT);
    m_pidController.setSmartMotionMinOutputVelocity(MIN_VELOCITY_RPM, SMART_MOTION_SLOT);
    m_pidController.setSmartMotionMaxAccel(MAX_ACCELERATION_RPM_PER_SEC, SMART_MOTION_SLOT);
    m_pidController.setSmartMotionAllowedClosedLoopError(ALLOWED_ERROR, SMART_MOTION_SLOT);
  
  }

  public static PivoterSubsystem getInstance() {
    if (instance == null) {
      instance =  new PivoterSubsystem();
    }
    return instance;
  }

  public void pivot(double speed) {
    leftPivoter.setInverted(true);
    rightPivoter.follow(leftPivoter, true);

    // Add input safety measure. 
    if(Math.abs(speed) > 1){
      System.out.println("SPEED TARGET EXCEEDS LIMIT");
    } else{
      if(speed < 0 && getLowerSwitchToggled()){
        // If we hit the switch and we're going down, stop the motor and reset the encoder
        resetEncoder(0);
        leftPivoter.set(0);
      }else if (speed > 0 && (getHigherSwitchToggled())) {
        // If we're going up and exceed our degrees for the pivoter, stop. 
        leftPivoter.set(0);
      } else {
        leftPivoter.set(speed);
      }  
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

  public double getRightPivoterRawRotation(){
    // Return the pivoter raw position in rotations. 
    return rightEncoder.getPosition();
  }


  public double getPivoterRotation(){
    // Return the pivoter raw position in rotations. 
    return getPivoterRawRotation() * PivoterConstants.kPivoterGearRatio;
  }

  public double getPivoterDegrees(){
    // 360 degrees in a rotation
    return getPivoterRotation() * 360; 
  }

  public double convertDegreesToMotorRotations(double degrees){
    degrees /= 360;
    return degrees / PivoterConstants.kPivoterGearRatio;
  }

  public void resetEncoder(double position){
    // Set the encoder back to normal
    leftEncoder.setPosition(position);
    rightEncoder.setPosition(position);

  }

  public Boolean getHigherSwitchToggled() {
    return !rightLimit.get();
  }

  public Boolean getLowerSwitchToggled(){
    //Return if any of the switches were toggled
    return !leftLimit.get();
  }

  // WE MAY NEED IT FOR NON-PID CONTROL
  public void addFeedFoward(){
    // Add some power to the pivoter to have it hold against gravity. 
    if(!getLowerSwitchToggled()){ // Make sure the trigger isn't activated. 
      pivot(0.05);
    }
  }


  // Method to run the motor to our target input using PID. 
  public void PIDPivot(double rotations){
    // Don't do anything if target is out of bounds.    
    leftPivoter.setInverted(true);
    rightPivoter.follow(leftPivoter, true);

    if ((rotations >= PivoterConstants.kMaxPivoterRotations) || rotations < -5){
      System.out.println("ABORT");
    }else{
      // Set our global target variable to be whatever we targeted it to be. 
      Constants.PivoterConstants.kPivoterTarget = rotations;
      
      m_pidController.setReference(
        rotations,
        CANSparkMax.ControlType.kSmartMotion,
        SMART_MOTION_SLOT,
        getArbFF()
      );  
    }
    SmartDashboard.putNumber("Target Rotation Pivot", rotations);
  }

  // "Extra" voltage needed 
  public double getArbFF() {
    double radians = Units.degreesToRadians(getPivoterDegrees());
    return Math.cos(radians) * horizontalArbFF; // We need "max" ff when degrees is 0
  }

  @Override
  public void periodic() {

    if(getLowerSwitchToggled()){
      resetEncoder(0);
    }else if(getHigherSwitchToggled()){
      resetEncoder(PivoterConstants.kMaxPivoterRotations);

    };


    // Print debug information
    SmartDashboard.putBoolean("4 - Pivotor Left Switch", getLowerSwitchToggled());
    SmartDashboard.putBoolean("5 - Pivotor Right Switch", getHigherSwitchToggled());
    // SmartDashboard.putBoolean("Pivotor Switch Input", getSwitchToggled());

    SmartDashboard.putNumber("Right Pivotor Raw Rotations", getRightPivoterRawRotation());

    SmartDashboard.putNumber("Pivotor Raw Rotations", getPivoterRawRotation());
    SmartDashboard.putNumber("Pivotor Rotations", getPivoterRotation());
    SmartDashboard.putNumber("Pivotor Degrees", getPivoterDegrees());


    SmartDashboard.putNumber("Left Pivotor Current", leftPivoter.getOutputCurrent());
    SmartDashboard.putNumber("Right Pivotor Current", rightPivoter.getOutputCurrent());

    // Read our global variable
    SmartDashboard.putNumber("GLOBAL: Pivoter Target", PivoterConstants.kPivoterTarget);

  }
}

// Gains Class:
class Gains {
  public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final double kIzone;
	public final double kPeakOutput;
	
	public Gains(double _kP, double _kI, double _kD, double _kF, double _kIzone, double _kPeakOutput){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
		kIzone = _kIzone;
		kPeakOutput = _kPeakOutput;
    }
}
