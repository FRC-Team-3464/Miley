// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.Random;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.PivoterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoPositionToby extends Command {
  /** Creates a new AutoPositionToby. */
  public Random rand;
  public int randomIndex;
  public final int[] availables = {5, 13, 12, 11, 10, 1};
  public static int tobyTag;
  private Transform3d camToTarget;
  public static double range;
  public static Rotation2d targetHeading;
  public static double CAMERA_TO_ROBOT_X = Units.inchesToMeters(9); // Robot Height
  public static double CAMERA_TO_ROBOT_Y = Units.inchesToMeters(2); // Distance from cross hair center to apriltag center (left-right)
  public static double CAMERA_TO_ROBOT_Z = Units.inchesToMeters(14.5); //Robot distance from center
  public static double TobyHeight = Units.inchesToMeters(50);
  public static double targetDistance = 2; //CHANGE THIS


  public static ProfiledPIDController rotationController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 
    0, 
    0, 
    AutoConstants.kThetaControllerConstraints);

  public static PIDController distanceController = new PIDController(
    AutoConstants.kPXController, 
    0, 
    0);

  private final PhotonCamera photonCamera;

  private final static SwerveSubsystem swerveSub = SwerveSubsystem.getInstance();
  private final PivoterSubsystem pivoterSub = PivoterSubsystem.getInstance();
  private final PhotonSubsystem photonSub = PhotonSubsystem.getInstance();

  
  public AutoPositionToby(int intTag) {
    rotationController.setTolerance(Units.degreesToRadians(3));
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    tobyTag = intTag;
    photonCamera = photonSub.getAprilCamera();
    addRequirements(swerveSub);
    addRequirements(photonSub);
    addRequirements(pivoterSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // targetHeading = new Rotation2d(0);
    // rand = new Random();
    // randomIndex = rand.nextInt(6);
    // tobyTag = availables[randomIndex];
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var photonRes = photonCamera.getLatestResult();

    if (photonRes.hasTargets()) {
      var targetOpt = photonRes.getTargets().stream()
        .filter(t -> t.getFiducialId() == tobyTag)
        .filter(t -> t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
        .findFirst();

      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        camToTarget = target.getBestCameraToTarget();
        var rotationDegrees = getRotationDegreesToSpeaker();

        range = PhotonUtils.calculateDistanceToTargetMeters(
          CAMERA_TO_ROBOT_X, 
          Units.inchesToMeters(TobyHeight), 
          Math.PI/6, 
          targetOpt.get().getPitch());
        
        distanceController.setSetpoint(targetDistance);
        double forwardSpeed = -distanceController.calculate(range);

        if (rotationDegrees != 0) {
          var drivetrainHeading = swerveSub.getRotation2d();

          targetHeading = drivetrainHeading.minus(Rotation2d.fromDegrees(rotationDegrees));
          swerveSub.driveRobotRelative(new ChassisSpeeds(forwardSpeed, 0, rotateToTag(targetHeading, swerveSub.getRotation2d())));
        }
      }
    }
    else {
      if (targetHeading.getRadians() != 0) {
        swerveSub.driveRobotRelative(ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, rotateToTag(targetHeading, swerveSub.getRotation2d()), swerveSub.getRotation2d()));
      }
      else {
        swerveSub.stopModules();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSub.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (camToTarget != null) {
      var rotationDegrees = getRotationDegreesToSpeaker();

      Rotation2d verifyTargetHeading = swerveSub.getRotation2d().minus(Rotation2d.fromDegrees(rotationDegrees));

      if (verifyTargetHeading != null) {
        var isRotationOnTarget = Math.abs(targetHeading.getDegrees() - swerveSub.getRotation2d().getDegrees()) < 3;

        // return (isRotationOnTarget && Math.abs(range - targetDistance) < 0.2);
          return (Math.abs(range - targetDistance) < 0.2);
      }
    }
    return false;
  }

  private double getRotationDegreesToSpeaker() {
    return Math.atan2(
      camToTarget.getY() + CAMERA_TO_ROBOT_Y,
      camToTarget.getX() + CAMERA_TO_ROBOT_X)
      * (180 / Math.PI);
  }

  private double rotateToTag(Rotation2d target, Rotation2d drivetrainHeading) {
    rotationController.setGoal(target.getRadians());
    var rotationSpeed = rotationController.calculate(drivetrainHeading.getRadians());
    if (rotationController.atGoal()) {
      rotationSpeed = 0;
    }
    return rotationSpeed;
  }
}
