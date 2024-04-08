package frc.robot.commands.Swerve;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.PivoterConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.PivoterSubsystem;

public class SwerveAimSpeaker extends Command {

  private static final int RED_SPEAKER_TAG = 4;
  private static final int BLUE_SPEAKER_TAG = 8;
  private static final double TAG_TO_SPEAKER_Z = 0.5;  //fixme: get real value
  public static double CAMERA_TO_ROBOT_X = Units.inchesToMeters(9); //fixme: get real value
  public static double CAMERA_TO_ROBOT_Y = Units.inchesToMeters(2); //fixme: get real value
  public static double CAMERA_TO_ROBOT_Z = Units.inchesToMeters(14.5); //fixme: get real value

// DEBUG: if we miss the apriltag - reference a previous angle
  public static double previousAngle = -1; 
  // Used to count how many times that we're off. 
  public static double previousAngleCounter = 0;
  

  public static Rotation2d targetHeading;

  public static final double ROTATION_DEGREES_TOLERANCE = 1;
  public static final double PIVOT_DEGREES_TOLERANCE = 1;
  private static final double AIM_TIME = 5;
  private final Timer aimTimer = new Timer();

  private final PhotonCamera photonCamera;
  // private final ArrayList<Boolean> success = new ArrayList<Boolean>();
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final PivoterSubsystem pivoterSub = PivoterSubsystem.getInstance();
  private final PhotonSubsystem photonSub = PhotonSubsystem.getInstance();
  private final LEDSubsystem ledSub = LEDSubsystem.getInstance();

  // private static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
  // private final ProfiledPIDController rotationController = new ProfiledPIDController(2, 0, 0, ROTATION_CONSTRAINTS);

    // Profiled PID Controller = PID Controller with constraints on max speed / acceleration. 
    public static ProfiledPIDController rotationController = new ProfiledPIDController(
      AutoConstants.kPThetaController,
      0,
      0,
      AutoConstants.kThetaControllerConstraints);
  
  private Transform3d camToTarget;
  ShuffleboardTab tab = Shuffleboard.getTab("Vision");

  public SwerveAimSpeaker() {
    // this.success = success;

    rotationController.setTolerance(Units.degreesToRadians(3));
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerveSubsystem);
    addRequirements(photonSub);
    addRequirements(pivoterSub);

    photonCamera = photonSub.getAprilCamera();

    tab.addString("CameraToTarget", this::getFomattedTransform3d).withPosition(0, 5).withSize(2, 2);
  }

  @Override
  public void initialize() {
    // Start counting seconds of aim-time
    camToTarget = null;
    targetHeading = new Rotation2d(0);
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
      
      // If we find an optimal target: 
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        camToTarget = target.getBestCameraToTarget();
        ledSub.setBlue();

        // Calculate how far off we are rotation-wise from facing the apriltag's center. 
        var rotationDegrees = getRotationDegreesToSpeaker();
        if (rotationDegrees != 0.0) {
          // Get where the swerve is heading
          var drivetrainHeading = swerveSubsystem.getRotation2d();
          
          // Update our target heading - calculate how much the robot needs to rotate to center at the apriltag.
          targetHeading = drivetrainHeading.minus(Rotation2d.fromDegrees(rotationDegrees));
          rotateToSpeaker(targetHeading, drivetrainHeading); 
          pivotToSpeaker();

        }
      }

    } else {
      // If no target found, see if we have a previous angle we can use as a reference. 
      if (targetHeading.getRadians() != 0){
        // System.out.print("USING PREVIOUS Target of:");
        // System.out.println(targetHeading.getRadians());
        ledSub.setYellow();

        // Rotate using our previous target
        rotateToSpeaker(targetHeading, swerveSubsystem.getRotation2d()); 
    
      } else {
        System.out.println("NO REF FOUND");
        // Reset previous angle to be undefined
        previousAngle = -1;
        swerveSubsystem.stopModules();
        ledSub.setRed();
        camToTarget = null;
      }
    }

  }

  // Set our controller's target to the targets radians, and have it turn 
  private void rotateToSpeaker(Rotation2d target, Rotation2d drivetrainHeading ) {
    rotationController.setGoal(target.getRadians());

    var rotationSpeed = rotationController.calculate(drivetrainHeading.getRadians());
    if (rotationController.atGoal()) {
      rotationSpeed = 0;
    }

    // Rotate using relative speeds. 
    swerveSubsystem.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rotationSpeed, drivetrainHeading));
    SmartDashboard.putNumber("Target Rotations", targetHeading.getRadians());
    SmartDashboard.putNumber("Current Swerve Rotations", drivetrainHeading.getRadians());  

  }    

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
    // pivoterSub.stopMotor();
    photonCamera.setLED(VisionLEDMode.kOff);

  }

  @Override
  public boolean isFinished() {
    // If the timer has elapsed for aiming - end the command
    if (aimTimer.hasElapsed(AIM_TIME)) {
      System.out.println("NO TARGET FOUND");
      return true;
    } 
    
    // Get our current target heading
    var drivetrainHeading = swerveSubsystem.getRotation2d();
    if (camToTarget != null) {
      var rotationDegrees = getRotationDegreesToSpeaker();
      Rotation2d verifyTargetHeading = drivetrainHeading.minus(Rotation2d.fromDegrees(rotationDegrees));
      
      // If we have a target heading, see if the error is within tolarance. 
      if (verifyTargetHeading != null) {
        var isRotationOnTarget = Math.abs(targetHeading.getDegrees() - drivetrainHeading.getDegrees()) < ROTATION_DEGREES_TOLERANCE;
        
        if (isRotationOnTarget){
          ledSub.setGreen();
        };

        return isRotationOnTarget;
    }
      // var pivotDegrees = getPivotDegreesToSpeaker();
      // var pivoterDegrees = pivoterSub.getPivoterDegrees();
      // var isPivoterOnTarget = Math.abs(pivoterDegrees - pivotDegrees) < PIVOT_DEGREES_TOLERANCE;
      // success[0] = isRotationOnTarget && isPivoterOnTarget;
      // return success[0];
    }
    
    return false;
  }


  private void pivotToSpeaker(){
    var pivotDegrees = getPivotDegreesToSpeaker();
    var pivotRotations = pivoterSub.convertDegreesToMotorRotations(pivotDegrees);

    if (pivotRotations < PivoterConstants.kMaxPivoterRotations && pivotRotations > 0) {
      // print()
      photonCamera.setLED(VisionLEDMode.kBlink);
    } else {
      photonCamera.setLED(VisionLEDMode.kOff);
    }

    // SmartDashboard.putNumber("Speaker Rotations", Units.degreesToRadians(rotationDegrees));

  }



  // private void pivotToSpeaker(){
  //   var pivotDegrees = getPivotDegreesToSpeaker();
  //   var pivotRotations = pivoterSub.degreesToMotorRotations(pivotDegrees);

  //   SmartDashboard.putNumber("Speaker Rotations", Units.degreesToRadians(rotationDegrees));
  //   SmartDashboard.putNumber("Target Rotations", targetHeading.getRadians());
  // }

  //   if (pivotRotations > PivoterConstants.kMaxPivoterRotations || pivotRotations < 0) {
  //     photonCamera.setLED(VisionLEDMode.kBlink);
  //   } else {
  //     pivoterSub.PIDPivot(pivotRotations);
  //     photonCamera.setLED(VisionLEDMode.kOff);
  //   }
  // }

  // Return the degree the camera makes with the apriltag - how much we're off from being dead center to the apriltag. 
  private double getRotationDegreesToSpeaker() {
    return Math.atan2(
      camToTarget.getY() + CAMERA_TO_ROBOT_Y,
      camToTarget.getX() + CAMERA_TO_ROBOT_X)
      * (180 / Math.PI);
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