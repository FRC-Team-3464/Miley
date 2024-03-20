package frc.robot.commands;

import static frc.robot.subsystems.PoseEstimatorSubsystem.ROBOT_TO_CAMERA;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivoterConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PivoterSubsystem;

public class AimSpeaker extends Command {

  private static final int RED_SPEAKER_TAG = 4;
  private static final int BLUE_SPEAKER_TAG = 8;
  private static final double TAG_TO_SPEAKER_Z = 0.5;  //fixme: get real value
  public static double CAMERA_TO_ROBOT_X = 0.15; //fixme: get real value
  public static double CAMERA_TO_ROBOT_Y = 0; //fixme: get real value
  public static double CAMERA_TO_ROBOT_Z = 0.15; //fixme: get real value

  private final PhotonCamera photonCamera;
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final PivoterSubsystem pivoterSub = PivoterSubsystem.getInstance();

  private final ProfiledPIDController rotationController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  private Transform3d camToTarget = new Transform3d();

  ShuffleboardTab tab = Shuffleboard.getTab("Vision");

  public AimSpeaker(PhotonCamera photonCamera) {
    this.photonCamera = photonCamera;

    rotationController.setTolerance(Units.degreesToRadians(3));
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerveSubsystem);
    addRequirements(pivoterSub);

    tab.addString("CameraToTarget", this::getFomattedTransform3d).withPosition(0, 5).withSize(2, 2);
  }

  @Override
  public void initialize() {
    camToTarget = null;
    rotationController.reset(swerveSubsystem.getRotation2d().getRadians());
  }

  @Override
  public void execute() {

    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag straight below speaker
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == RED_SPEAKER_TAG || t.getFiducialId() == BLUE_SPEAKER_TAG)
          .filter(t -> t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
          .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        camToTarget = target.getBestCameraToTarget();
        rotateToSpeaker(); //fixme: parallelize
        pivotToSpeaker();
        photonCamera.setLED(VisionLEDMode.kOn);
      }
    } else {
      camToTarget = null;
      photonCamera.setLED(VisionLEDMode.kOff);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
    pivoterSub.stopMotor();
    photonCamera.setLED(VisionLEDMode.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;  //fixme: when to end?
  }

  private void rotateToSpeaker() {
      var rotation = getRotationToSpeaker();

    if (rotation != 0.0) {
      var drivetrainHeading = swerveSubsystem.getRotation2d();
      var targetHeading = drivetrainHeading.plus(rotation);
      rotationController.setGoal(targetHeading.getRadians());
      var rotationSpeed = rotationController.calculate(drivetrainHeading.getRadians());
      swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rotationSpeed, drivetrainHeading));
  }

  private void pivotToSpeaker(){
    var degrees = getDegreesToSpeaker();
    var rotations = pivoterSub.degreesToMotorRotations(degrees);

    if (rotations > PivoterConstants.kMaxPivoterRotations || rotations < 0) {
      photonCamera.setLED(VisionLEDMode.kBlink);
    } else {
      pivoterSub.PIDPivot(rotations);
      photonCamera.setLED(VisionLEDMode.kOff);
    }
  }

  private double getRotationToSpeaker() {
    return Math.atan2(
      camToTarget.getY() + CAMERA_TO_ROBOT_Y,
      camToTarget.getX() + CAMERA_TO_ROBOT_X)
      * 180 / Math.PI;
  }

  private double getDegreesToSpeaker() {
    return Math.atan2(
      camToTarget.getZ() + CAMERA_TO_ROBOT_Z + TAG_TO_SPEAKER_Z,
      Math.sqrt(Math.pow(camToTarget.getX() + CAMERA_TO_ROBOT_X, 2) + Math.pow(camToTarget.getY() + CAMERA_TO_ROBOT_Y, 2)))
      * 180 / Math.PI;
  }

  private String getFomattedTransform3d() {
    var transform3d = camToTarget;
    return transform3d == null ? "" : String.format("(%.2f, %.2f, %.2f)",
        transform3d.getX(),
        transform3d.getY(),
        transform3d.getZ());
  }

}
