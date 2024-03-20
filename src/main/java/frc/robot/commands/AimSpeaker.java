package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
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

  public static final double ROTATION_DEGREES_TOLERANCE = 1;
  public static final double PIVOT_DEGREES_TOLERANCE = 1;
  private static final double AIM_TIME = 3;
  private final Timer aimTimer = new Timer();

  private final PhotonCamera photonCamera;
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final PivoterSubsystem pivoterSub = PivoterSubsystem.getInstance();

  private static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
  private final ProfiledPIDController rotationController = new ProfiledPIDController(2, 0, 0, ROTATION_CONSTRAINTS);

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
    aimTimer.reset();
    aimTimer.start();
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

    if (aimTimer.hasElapsed(AIM_TIME)) {
      return true;
    } else if (camToTarget != null) {
      var rotationDegrees = getRotationDegreesToSpeaker();
      var isRotationOnTarget = Math.abs(rotationDegrees) < ROTATION_DEGREES_TOLERANCE;
      var pivotDegrees = getPivotDegreesToSpeaker();
      var pivoterDegrees = pivoterSub.getPivoterDegrees();
      var isPivoterOnTarget = Math.abs(pivoterDegrees - pivotDegrees) < PIVOT_DEGREES_TOLERANCE;
      return isRotationOnTarget && isPivoterOnTarget;      
    }
    return false;
  }

  private void rotateToSpeaker() {
      var rotationDegrees = getRotationDegreesToSpeaker();

    if (rotationDegrees != 0.0) {
      var drivetrainHeading = swerveSubsystem.getRotation2d();
      var targetHeading = drivetrainHeading.plus(Rotation2d.fromDegrees(rotationDegrees));
      rotationController.setGoal(targetHeading.getRadians());
      var rotationSpeed = rotationController.calculate(drivetrainHeading.getRadians());
      swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rotationSpeed, drivetrainHeading));
    }
  }

  private void pivotToSpeaker(){
    var pivotDegrees = getPivotDegreesToSpeaker();
    var pivotRotations = pivoterSub.degreesToMotorRotations(pivotDegrees);

    if (pivotRotations > PivoterConstants.kMaxPivoterRotations || pivotRotations < 0) {
      photonCamera.setLED(VisionLEDMode.kBlink);
    } else {
      pivoterSub.PIDPivot(pivotRotations);
      photonCamera.setLED(VisionLEDMode.kOff);
    }
  }

  private double getRotationDegreesToSpeaker() {
    return Math.atan2(
      camToTarget.getY() + CAMERA_TO_ROBOT_Y,
      camToTarget.getX() + CAMERA_TO_ROBOT_X)
      * 180 / Math.PI;
  }

  private double getPivotDegreesToSpeaker() {
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
