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

public class AimSpeakerCommand extends Command {

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

  private static final int RED_SPEAKER_TAG = 4;
  private static final int BLUE_SPEAKER_TAG = 8;
  private static final double TAG_TO_SPEAKER_Z = 0.5;  //fixme: get real value
  public static double CAMERA_TO_ROBOT_X = 0.15; //fixme: get real value
  public static double CAMERA_TO_ROBOT_Y = 0; //fixme: get real value
  public static double CAMERA_TO_ROBOT_Z = 0.15; //fixme: get real value

  private final PhotonCamera photonCamera;
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final PivoterSubsystem pivoterSub = PivoterSubsystem.getInstance();

  private Transform3d camToTarget = new Transform3d();

  ShuffleboardTab tab = Shuffleboard.getTab("Vision");

  public AimSpeakerCommand(PhotonCamera photonCamera) {
    this.photonCamera = photonCamera;
    addRequirements(swerveSubsystem);
    addRequirements(pivoterSub);

    tab.addString("CameraToTarget", this::getFomattedTransform3d).withPosition(0, 5).withSize(2, 2);
  }

  @Override
  public void initialize() {
    camToTarget = null;
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
        photonCamera.setLedMode(VisionLEDMode.kOn);
      }
    } else {
      camToTarget = null;
      photonCamera.setLedMode(VisionLEDMode.kOff);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
    pivoterSub.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;  //fixme: when to end?
  }

  private void rotateToSpeaker() {
    var rotation = getRotationToSpeaker();

    if (rotation != 0.0)
      swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rotation, swerveSubsystem.getRotation2d()));
  }

  private void pivotToSpeaker(){
    var degrees = getDegreesToSpeaker();
    var rotations = pivoterSub.degreesToMotorRotations(degrees);

    if (rotations > PivoterConstants.kMaxPivoterRotations){
      rotations = PivoterConstants.kMaxPivoterRotations;
    } else if (rotations < 0){
      rotations = 0;
    }

    pivoterSub.PIDPivot(rotations);
  }

  private double getRotationToSpeaker() {
    return Math.atan2(
      camToTarget.getY() + CAMERA_TO_ROBOT_Y,
      camToTarget.getX() + CAMERA_TO_ROBOT_X)
      * 180 / Math.PI;
  }

  private double getDegreesToSpeaker() {
    var degrees = Math.atan2(
      camToTarget.getZ() + CAMERA_TO_ROBOT_Z,
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
