// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorConstants;

public class SwerveSubsystem extends SubsystemBase {
  // Define the four modules
  private final SwerveModule frontLeft = new SwerveModule(
    MotorConstants.kFrontLeftDriveMotorPort,
    MotorConstants.kFrontLeftTurningMotorPort,
    DriveConstants.kFrontLeftDriveEncoderReversed,
    DriveConstants.kFrontLeftTurningEncoderReversed,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

  private final SwerveModule frontRight = new SwerveModule(
    MotorConstants.kFrontRightDriveMotorPort,
    MotorConstants.kFrontRightTurningMotorPort,
    DriveConstants.kFrontRightDriveEncoderReversed,
    DriveConstants.kFrontRightTurningEncoderReversed,
    DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

  private final SwerveModule backLeft = new SwerveModule(
    MotorConstants.kBackLeftDriveMotorPort,
    MotorConstants.kBackLeftTurningMotorPort,
    DriveConstants.kBackLeftDriveEncoderReversed,
    DriveConstants.kBackLeftTurningEncoderReversed,
    DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

  private final SwerveModule backRight = new SwerveModule(
    MotorConstants.kBackRightDriveMotorPort,
    MotorConstants.kBackRightTurningMotorPort,
    DriveConstants.kBackRightDriveEncoderReversed,
    DriveConstants.kBackRightTurningEncoderReversed,
    DriveConstants.kBackRightDriveAbsoluteEncoderPort,
    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private static SwerveSubsystem instance = null;  

  // Odemeter to track robot position and create feedback speeds in auto. 
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics, 
    getRotation2d(),
    new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
  });


  private final Field2d field;
  
  public SwerveSubsystem() {
    // In the constructer, wait a second, then update the gyro (it will have been booted up by then).
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
    
      }
    }).start();

    // Put our field onto smartdashboard. 
    field = new Field2d();
    SmartDashboard.putData("Field", field);

    
  AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    // CHECK - REUSED FROM OLD AUTO PATH. 
                    new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // Rotation PID constants
                    // 0.43105229381
                    5, // Max module speed, in m/s
                    0.43105229381, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return !(alliance.get() == DriverStation.Alliance.Red);
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

  }

  // PATHPLANNER HELPER FUNCTIONS
  // Returns position of robot (x, y, theta) using meters. 
  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }
  
  // Reset the odometry with the rotation, positions, and current pose. 
  // Reference code: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(
      getRotation2d(), 
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      },
      pose /* Where the robot is on the field */);  
  }


  
  public ChassisSpeeds getCurrentSpeeds() {
    /*  NEED TO CHECK THIS
    * Reference; 
    * https://github.com/3015RangerRobotics/2023Public/blob/main/RobotCode2023/src/main/java/frc/robot/subsystems/Swerve.java
    */
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      new SwerveModuleState[] {
          frontLeft.getState(),
          frontRight.getState(),
          backLeft.getState(),
          backRight.getState()
      });
  }

  /*
  * The following commands are used for pathplanner: driveRobotRelative and getCurrentSpeeds()
  * 
  */
  // Used for pathplanner: https://pathplanner.dev/pplib-build-an-auto.html#load-an-auto
  public void driveRobotRelative(ChassisSpeeds speeds) {
    /*  NEED TO CHECK THIS
    * Reference; 
    * https://github.com/3015RangerRobotics/2023Public/blob/main/RobotCode2023/src/main/java/frc/robot/subsystems/Swerve.java
    */
    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
  
    // SHOULD Desaturate speeds for swerve. 
    setModuleStates(targetStates);
  }




  public static SwerveSubsystem getInstance() {
    if (instance == null) {
      instance = new SwerveSubsystem();
    }
    return instance;
  }


  public void zeroHeading () {
    gyro.reset();
  }

  // Take the of the gyro by taking the reminder of the reading divided by 360  
  public double getHeading () {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }
  
  // Translate the heading to a 2D rotation object. 
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading()); 
  }
  

  
  @Override
  public void periodic() {
    // Update our odometer with rotation and wheel positions (rotation and drive position).
    odometer.update(
      getRotation2d(),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    });

    // SmartDashboard.putBoolean("Red Alliance?", (DriverStation.getAlliance().get() == Alliance.Red));
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

    // SmartDashboard.putNumber("Front Left Absolute Encoder", Units.rotationsToDegrees(frontLeft.getAbsoluteEncoderRot()));
    // SmartDashboard.putNumber("Front Right Absolute Encoder",  Units.rotationsToDegrees(frontRight.getAbsoluteEncoderRot()));
    // SmartDashboard.putNumber("Back Left Absolute Encoder",  Units.rotationsToDegrees(backLeft.getAbsoluteEncoderRot()));
    // SmartDashboard.putNumber("Back Right Absolute Encoder",  Units.rotationsToDegrees(backRight.getAbsoluteEncoderRot()));

    SmartDashboard.putNumber("Front Left Absolute Encoder", frontLeft.getAbsoluteEncoderRot());
    SmartDashboard.putNumber("Front Right Absolute Encoder",  frontRight.getAbsoluteEncoderRot());
    SmartDashboard.putNumber("Back Left Absolute Encoder",  backLeft.getAbsoluteEncoderRot());
    SmartDashboard.putNumber("Back Right Absolute Encoder", backRight.getAbsoluteEncoderRot());



    // SmartDashboard.putNumber("Front Left Relative Encoder", frontLeft.getTurningPosition());
    SmartDashboard.putNumber("Front Left Relative Encoder", Units.radiansToDegrees(frontLeft.getTurningPosition()));
    SmartDashboard.putNumber("Front Right Relative Encoder", Units.radiansToDegrees(frontRight.getTurningPosition()));
    SmartDashboard.putNumber("Back Left Relative Encoder", Units.radiansToDegrees(backLeft.getTurningPosition()));
    SmartDashboard.putNumber("Back Right Relative Encoder", Units.radiansToDegrees(backRight.getTurningPosition()));


    SmartDashboard.putNumber("Front Left Drive Current", frontLeft.getDriveCurrent());
    SmartDashboard.putNumber("Front Right Drive Current", frontRight.getDriveCurrent());
    SmartDashboard.putNumber("Back Left Drive Current", backLeft.getDriveCurrent());
    SmartDashboard.putNumber("Back Right Drive Current", backRight.getDriveCurrent());
    
    SmartDashboard.putNumber("Front Left Turn Current", frontLeft.getTurnCurrent());
    SmartDashboard.putNumber("Front Right Turn Current", frontRight.getTurnCurrent());
    SmartDashboard.putNumber("Back Left Turn Current", backLeft.getTurnCurrent());
    SmartDashboard.putNumber("Back Right Turn Current", backRight.getTurnCurrent());


    // SmartDashboard.putNumber("Front Left Drive Position", frontLeft.getDrivePosition());
    // SmartDashboard.putNumber("Front Right Drive Position", frontRight.getDrivePosition());
    // SmartDashboard.putNumber("Back Left Drive Position", backLeft.getDrivePosition());
    // SmartDashboard.putNumber("Back Right Drive Position", backRight.getDrivePosition());


    // Update our robot's position so we can see it too. 
    field.setRobotPose(getPose());

  }

public void stopModules(){
  frontLeft.stop();
  frontRight.stop();
  backLeft.stop();
  backRight.stop();
}

public void setModuleStates(SwerveModuleState[] desiredStates) {
  // Update desired speeds to be constrained to a max speed. 
  // https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-template-2021-unmaintained/issues/8

  // Makes sure that the speeds are below the max while keeping their relative ratios. 
  SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond); // chiefdelphi.com/t/normalizewheelspeeds/411155
  frontLeft.setDesiredState(desiredStates[0]);
  frontRight.setDesiredState(desiredStates[1]);
  backLeft.setDesiredState(desiredStates[2]);
  backRight.setDesiredState(desiredStates[3]);
}
}