// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


public class SwerveAutoDistance extends Command {
  /** Creates a new SwerveAutoDistance. */
  private static final int RED_SPEAKER_TAG = 4;
  private static final int BLUE_SPEAKER_TAG = 8;
  private static final double TAG_TO_SPEAKER_Z = 0.5;  //fixme: get real value
  public static double CAMERA_TO_ROBOT_X = Units.inchesToMeters(9); //fixme: get real value
  public static double CAMERA_TO_ROBOT_Y = Units.inchesToMeters(2); //fixme: get real value
  public static double CAMERA_TO_ROBOT_Z = Units.inchesToMeters(14.5); //fixme: get real value

  public static double TARGET_DISTANCE = 20; // Fixme: get real value
  public static double pivoterTargetRotation = 10; // Fixme: get real value
  

  private static final double POSITION_TOLERANCE = 0.1;
  private static final double AIM_TIME = 5;
  private final Timer aimTimer = new Timer();
  

  private final PhotonCamera photonCamera;
  private final SwerveSubsystem swerveSub = SwerveSubsystem.getInstance();
  private final PhotonSubsystem photonSub = PhotonSubsystem.getInstance();

  // Rotation2d drivetrainHeading = swerveSub.getRotation2d();
  private final LEDSubsystem ledSub = LEDSubsystem.getInstance();

  private final PIDController distancePidController = new PIDController(AutoConstants.kPXController, 0, 0);


  public SwerveAutoDistance() {
    photonCamera = photonSub.getAprilCamera();
    distancePidController.setTolerance(POSITION_TOLERANCE);

    aimTimer.start();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSub);
    addRequirements(photonSub);
    addRequirements(ledSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = photonCamera.getLatestResult();
    // using height of target and angle of the camera and pitch from target, we use tan to calculate the distance
    // I say tan(theta) = opp / adj, so adj = opp / tan(theta); we know what opp and theta (pitch offset + target pitch) are too :)
    if(result.hasTargets()) {
      var targetOpt = result.getTargets().stream()
      .filter(t -> t.getFiducialId() == RED_SPEAKER_TAG || t.getFiducialId() == BLUE_SPEAKER_TAG)
      .filter(t -> t.getPoseAmbiguity() <= 0.2 && t.getPoseAmbiguity() != -1)
      .findFirst();
      
      if(targetOpt.isPresent()) {
        double range = PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_TO_ROBOT_X, 
            Units.inchesToMeters(59.65), 
            Math.PI/6, 
            targetOpt.get().getPitch());

        System.out.println(range);
        SmartDashboard.putNumber("Apriltag Distance", Units.metersToInches(range));

        // distancePidController.setSetpoint(TARGET_DISTANCE);
        // double forwardPIDSpeed = distancePidController.calculate(range);
        // // swerveSub.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(
        // //   forwardPIDSpeed, 
        // //   0, 
        // //   0, 
        // //   // get our swerve heading as the robot relative heading. 
        // //   swerveSub.getRotation2d()));
      }
    }
    SmartDashboard.putBoolean("Target?", result.hasTargets());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledSub.setPurple();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (aimTimer.get() > AIM_TIME);
  }
}
