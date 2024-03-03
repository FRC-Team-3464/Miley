// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    // Level1 Standard Gear Ratios
    // https://www.andymark.com/products/mk4i-swerve-modules
    public static final double kDriveMotorGearRatio = 1 / 8.14;
    public static final double kTurningMotorGearRatio = 1 / 21.43; // 150/7 : 1 gear ratio
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
  }

  public static final class DriveConstants {
    public static final double kTrackWidth = Units.inchesToMeters(30);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(30);
  
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final int kFrontLeftDriveMotorPort = 8;
    public static final int kBackLeftDriveMotorPort = 2;
    public static final int kFrontRightDriveMotorPort = 6;
    public static final int kBackRightDriveMotorPort = 4;

    public static final int kFrontLeftTurningMotorPort = 7;
    public static final int kBackLeftTurningMotorPort = 1;
    public static final int kFrontRightTurningMotorPort = 5;
    public static final int kBackRightTurningMotorPort = 3;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false; // Check here

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 20;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 17;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 19;
    public static final int kBackRightDriveAbsoluteEncoderPort = 18;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.24462890625; // Reverse to right orientation
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.257080078125;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.49853515625-0.5;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -0.39697265625;

    // public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.2531 - 0.5 + 0.25; // Reverse to right orientation
    // public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.2568;
    // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.4924 + 0.5 - 0.25;
    // public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -0.3937;




    // Robot Speed Constraints
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.25;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
            kPhysicalMaxAngularSpeedRadiansPerSecond / 2.75;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
}

public static final class AutoConstants {
  public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
  public static final double kMaxAngularSpeedRadiansPerSecond = //
          DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 3;
  public static final double kPXController = 1.5;
  public static final double kPYController = 0.9;
  public static final double kPThetaController = 3.75 ;

  public static PIDController xController = new PIDController(kPXController, 0, 0);
  public static PIDController yController = new PIDController(kPYController, 0, 0);

  public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
          new TrapezoidProfile.Constraints(
                  kMaxAngularSpeedRadiansPerSecond,
                  kMaxAngularAccelerationRadiansPerSecondSquared);

    // Profiled PID Controller = PID Controller with constraints on max speed / acceleration. 
  public static ProfiledPIDController thetaController = new ProfiledPIDController(
    kPThetaController,
    0,
    0,
    kThetaControllerConstraints);

  public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveConstants.kDriveKinematics);

}


public static final class TragConstants {
  /*
   * 
   * AMP Coordinates
   * These are set to be on the red alliance
   */
  
   public static final Double[] coordsOriginToAmpStart = {0.0, 0.0, -90.0};
   public static final Double[] coordsOriginToAmpEnd = {0.427, 0.551, -90.0};
  //  public static final Double[] coordsOriginToAmpEnd = {0.427, 0.451, -90.0};

   public static final Double[] coordsAmpToAmpNoteStart = {0.0, 0.0, -90.0};   
   public static final Double[] coordsAmpToAmpNoteWayPoint1 = {0.9, -0.6, -40.0};
  //  public static final Double[] coordsAmpToAmpNoteWayPoint1 = {0.9, -0.6, -40.0};   
  // public static final Double[] coordsAmpToAmpNoteEnd = {1.13, -0.84, -40.0};

   public static final Double[] coordsAmpToAmpNoteEnd = {1.03, -0.74, -40.0};

   public static final Double[] coordsAmpToSpeakerNoteStart = {0.0, 0.0, -90.0};    
   public static final Double[] coordsAmpToSpeakerNoteWayPoint1 = {0.5, -2.178, -90.0};
   public static final Double[] coordsAmpToSpeakerNoteEnd = {1.055, -2.178, 0.0};

   public static final Double[] coordsAmpToHailMaryNoteStart = {0.0, 0.0, -90.0};    
   public static final Double[] coordsAmpToHailMaryNoteWayPoint1 = {5.5, -0.282, 0.0};
   public static final Double[] coordsAmpToHailMaryNoteEnd = {6.424, -0.282, 0.0};

  //  This is manually written because of an issue where the robot goes into the walls repeatedly
   public static final Double[] coordsHailMaryNoteToAmpStart = {coordsAmpToHailMaryNoteStart[0], coordsAmpToHailMaryNoteStart[1], coordsAmpToHailMaryNoteEnd[2]};
   public static final Double[] coordsHailMaryNoteToAmpWayPoint1 = {-coordsAmpToHailMaryNoteEnd[0], coordsAmpToHailMaryNoteEnd[1] * 2, 180.0};
   public static final Double[] coordsHailMaryNoteToAmpEnd = {-coordsAmpToHailMaryNoteEnd[0], -coordsAmpToHailMaryNoteEnd[1], 270.0};

  /*
   * 
   * Speaker Coordinates
   * Set to Red Alliance
   * 
   */

   public static final Double[] coordsOriginToStageNoteStart = {0.0, 0.0, 0.0};
   public static final Double[] coordsOriginToStageNoteStartWayPoint1 = {0.508, -1.4478, 0.0};
   public static final Double[] coordsOriginToStageNoteEnd = {1.3462, -1.4478, 0.0};

   public static final Double[] coordsStageNoteToSpeakerShootingStart = {0.0, 0.0, 0.0};
  //  public static final Double[] coordsStageNoteToSpeakerShootingWayPoint1 = {-0.508, 1.4478, 0.0};
   public static final Double[] coordsStageNoteToSpeakerShootingEnd = {-1.3462, 1.5, 0.0};

   public static final Double[] coordsSpeakerShootingToSpeakerNoteStart = {0.0, 0.0, 0.0};
  //  public static final Double[] coordsSpeakerShootingToSpeakerNoteEnd = {0.6, 0.0, 0.0};
  //  public static final Double[] coordsSpeakerShootingToSpeakerNoteEnd = {1.1, 0.0, 0.0};
   public static final Double[] coordsSpeakerShootingToSpeakerNoteEnd = {1.42, 0.0, 0.0};
  
  
  //  NEW - DRIVE Back  to origin
   public static final Double[] coordsSpeakerNoteToOriginStart = {0.0, 0.0, 0.0};
  //  public static final Double[] coordsSpeakerNoteToOriginEnd = {-1.1, 0.0, 0.0};
  public static final Double[] coordsSpeakerNoteToOriginEnd = {-1.42, 0.0, 0.0};
   


   public static final Double[] coordsSpeakerNoteToAmpShootingStart = {0.0, 0.0, 0.0};
   public static final Double[] coordsSpeakerNoteToAmpShootingEnd = {-0.508, 1.4478, 0.0};

   public static final Double[] coordsAmpNoteRotateToSpeakerStart = {0.0, 0.0, 0.0};
   public static final Double[] coordsAmpNoteRotateToSpeakerEnd = {-0.1, 0.0, 25.0};


  
    /*
     * 2 Note Center Hail Mary
     * 
     */
    public static final Trajectory tragOriginToFarCenterNote = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(-45)), 
        new Pose2d(6.9342, -3.4798, Rotation2d.fromDegrees(0))), AutoConstants.trajectoryConfig); // change X to 1.3464 because team spirit and nationalism 

    /* SHOULD BE TESTED */
    public static final Trajectory tragBlueOriginToFarCenterNote = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(45)), 
        new Pose2d(6.9342, 3.4798, Rotation2d.fromDegrees(0))), AutoConstants.trajectoryConfig); // change X to 1.3464 because team spirit and nationalism 

}


public static final class PivoterConstants{
  public static final double kMaxPivoterDegrees = 100;
  public static final double kSubwofferPivoterRotations = 1.7777; // kMaxPivoterDegrees / (360 * kPivoterGearRatio)
  // public static final double kAmpPivoterRotations = 28.5;
  
  public static final double kAmpPivoterRotations = 29;

  public static final double kStagePivoterRotations = 8.5;

  public static final double kMaxPivoterRotations = 31; // kMaxPivoterDegrees / (360 * kPivoterGearRatio)



  // Our gear ratio is as follows: (1/64) * (16/26)
  public static final double kPivoterGearRatio = 0.00961538462;
}

public static final class ElevatorConstants {
  public static final double kElevatorLowerSpeed = -1;
  public static final double kElevatorRaiseSpeed = 1;
}

public static final class OIConstants {
  public static final int kDriverControllerPort = 0;

  public static final int kDriverYAxis = 1;
  public static final int kDriverXAxis = 0;
  public static final int kDriverRotAxis = 4;
  public static final int kDriverFieldOrientedButtonIdx = 1;

  public static final double kDeadband = 0.08;
}

public static final class SandwichConstants {
  public static final double kIntakeSpeed = 0.5;
  public static final double kReverseIntakeSpeed = -1;

  public static final double kReverseShootSpeed = -1;
  public static final double kAmpShootSpeed = 0.5;
  public static final double kSpeakerShootSpeed = 1;

  public static final double kShootVelocityTarget = 5000;
}

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final Joystick auxStick = new Joystick(0); 
    public static final XboxController xbox = new XboxController(2);

    public static final JoystickButton button1 = new JoystickButton(auxStick, 1);
    public static final JoystickButton button2 = new JoystickButton(auxStick, 2);
    public static final JoystickButton button3 = new JoystickButton(auxStick, 3);
    public static final JoystickButton button4 = new JoystickButton(auxStick, 4);
    public static final JoystickButton button5 = new JoystickButton(auxStick, 5);
    public static final JoystickButton button6 = new JoystickButton(auxStick, 6);
    public static final JoystickButton button7 = new JoystickButton(auxStick, 7);
    public static final JoystickButton button8 = new JoystickButton(auxStick, 8);
    public static final JoystickButton button9 = new JoystickButton(auxStick, 9);
    public static final JoystickButton button10 = new JoystickButton(auxStick, 10);
    public static final JoystickButton button11 = new JoystickButton(auxStick, 11);
    public static final JoystickButton button12 = new JoystickButton(auxStick, 12);

    public static final POVButton pancakeUp = new POVButton(auxStick, 0);
    public static final POVButton pancakeDown = new POVButton(auxStick, 180);
    public static final POVButton pancakeRight = new POVButton(auxStick, 90);
    public static final POVButton pancakeLeft = new POVButton(auxStick, 270);
    
    public static final JoystickButton buttonX = new JoystickButton(xbox, 3);
    public static final JoystickButton buttonY = new JoystickButton(xbox, 4);
 
    //hello this is a test yahoooohoiuawhoi
  }
}
