// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveAimNote extends Command {
  private final PhotonCamera noteCamera;
  private final SwerveSubsystem swerveSub = SwerveSubsystem.getInstance();
  private final PhotonSubsystem photonSub = PhotonSubsystem.getInstance();

  private static Rotation2d targetHeading;
  private final Timer aimTimer = new Timer();
  public static ProfiledPIDController rotationController = new ProfiledPIDController(
    AutoConstants.kPThetaController,
    0,
    0,
    AutoConstants.kThetaControllerConstraints);

  public SwerveAimNote() {
    // Get the note camera from the photonsub. 
    noteCamera = photonSub.getNoteCamera();

    // Set our tolarence to be the equivolent of +-3 degrees
    rotationController.setTolerance(Units.degreesToRadians(3));
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(photonSub);
    addRequirements(swerveSub);
  }

  @Override
  public void initialize() {
    // Set our target to 0
    targetHeading = new Rotation2d(0);
    rotationController.reset(swerveSub.getRotation2d().getRadians());

    // Probably will not here...
    aimTimer.reset();
    aimTimer.start();
  }


  @Override
  public void execute() {
    // Get the latest result from the camera
    var result = noteCamera.getLatestResult();
    if (result.hasTargets()) { // See if there are any targets within. 
      var bestTarget = result.getBestTarget();
      var targetRotation = bestTarget.getYaw();


      // Get oru robot's current swerve heading. 
      var driveHeading = swerveSub.getRotation2d();

      if (targetRotation != 0) {
        // Set our target to be rotation needed to turn to note 
        targetHeading = Rotation2d.fromDegrees(targetRotation);
        rotationController.setGoal(Units.degreesToRadians(targetRotation));
        var rotationSpeed = rotationController.calculate(driveHeading.getRadians());
        if (rotationController.atGoal()) {
          rotationSpeed = 0;
        }
        var driveSpeed = -Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * (1 - (Math.abs(targetHeading.getDegrees()) / 28)) * 0.5;
        
        swerveSub.driveRobotRelative(ChassisSpeeds.fromRobotRelativeSpeeds(driveSpeed, 0, rotationSpeed, driveHeading));
      }
    }
    else { // If we don't have a camera target 
      // if we had a previous target - keep it as the target and follow the same logic
      if (targetHeading.getRadians() != 0) {
        rotationController.setGoal(targetHeading.getRadians());
        var rotationSpeed = rotationController.calculate(swerveSub.getRotation2d().getRadians());
        
        if (rotationController.atGoal()) {
          rotationSpeed = 0;
        }

        var driveSpeed = -Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * (1 - (Math.abs(targetHeading.getDegrees()) / 28)) * 0.5;
        swerveSub.driveRobotRelative(ChassisSpeeds.fromRobotRelativeSpeeds(driveSpeed, 0, rotationSpeed, swerveSub.getRotation2d()));
      }
      else {  // if we have no target + previous target
        System.out.println("NOPE!! NAH!!!! THERE'S NOTHING!!! EAOIGCFIEJFOIJNOIJ dang it");
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
    // If the turn timer has elapsed - meaning that we've spent too long aiming. 
    if (aimTimer.hasElapsed(5)) {
      System.out.println("NO TARGET FOUND LMAO UR AN IDIOT ARGH!!!");
      return true;
    }

    // If we have a target + controller is at target
    if (noteCamera.getLatestResult().hasTargets()) {
      if (rotationController.atGoal()) { 
  
          System.out.println("WOOHOO CONGRATULATIONS THIS ACTUALLY WORKED WTF?!?");
          return true;
      }
    }
    return false;
  }
}
