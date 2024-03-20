// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax shooterMotor = new CANSparkMax(11, MotorType.kBrushless);
  private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
  private final XboxController controller = new XboxController(2);

  private SparkPIDController shooterPidController = shooterMotor.getPIDController();

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  private static ShooterSubsystem instance = null;

  public ShooterSubsystem() {
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setInverted(false); 
    // shooterMotor.setSmartCurrentLimit(30);
//   PID coefficients (ง'̀-'́)ง
    kP = 0.000;
    kI = 0;
    kD = 0;
    kIz = 0;
    // kFF = 0.00019;

    kFF = 0.0002;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    // me: ლ(ಠ益ಠლ)

    shooterPidController.setP(kP);
    shooterPidController.setI(kI);
    shooterPidController.setD(kD);
    shooterPidController.setIZone(kIz);
    shooterPidController.setFF(kFF);
    shooterPidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
    //  \( ᐖ)/
    // i love this thing
  }

  // Note: Shooting must have the motor turning clockwise
  public void startPIDthings() {
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { shooterPidController.setP(p); kP = p; }
    if((i != kI)) { shooterPidController.setI(i); kI = i; }
    if((d != kD)) { shooterPidController.setD(d); kD = d; }
    if((iz != kIz)) {shooterPidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { shooterPidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      shooterPidController.setOutputRange(min, max); 
      kMinOutput = min; 
      kMaxOutput = max; 
    }
  }

  public double getVelocity() {
    return shooterEncoder.getVelocity();
  }
  public void runShooterPID(double setPoint) {
    shooterPidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
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

  public void rumbleController(double rumbleStrength) {
    controller.setRumble(RumbleType.kBothRumble, rumbleStrength);
  }

  public void stopRumble() {
    controller.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Velocity", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter F", shooterMotor.getPIDController().getFF());
    SmartDashboard.putNumber("Shooter Current", shooterMotor.getOutputCurrent());

    // This method will be called once per scheduler run
  }
}
