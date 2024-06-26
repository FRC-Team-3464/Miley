package frc.robot.commands.Swerve;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
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
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.PivoterConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.PivoterSubsystem;

public class SwerveAimAndPivot extends Command {
  private static final int RED_SPEAKER_TAG = 4;
  private static final int BLUE_SPEAKER_TAG = 7;
  // private static final double TAG_TO_SPEAKER_Z = 0.5;  //fixme: get real value

  private final PhotonCamera photonCamera;
  public static double CAMERA_TO_ROBOT_X = Units.inchesToMeters(9); // Robot Height
  public static double CAMERA_TO_ROBOT_Y = Units.inchesToMeters(2); // Distance from cross hair center to apriltag center (left-right)
  public static double CAMERA_TO_ROBOT_Z = Units.inchesToMeters(14.5); //Robot distance from center

  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final PivoterSubsystem pivoterSub = PivoterSubsystem.getInstance();
  private final PhotonSubsystem photonSub = PhotonSubsystem.getInstance();
  private final LEDSubsystem ledSub = LEDSubsystem.getInstance();

  // Profiled PID Controller = PID Controller with constraints on max speed + acceleration. 
  public static ProfiledPIDController rotationController = new ProfiledPIDController(
    AutoConstants.kPThetaController,
    0,
    0,
    AutoConstants.kThetaControllerConstraints);

  private Transform3d camToTarget;
  ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
  

  // Timer that tracks our aiming time. 
  private static final double AIM_TIME = 5;
  private final Timer aimTimer = new Timer();
    
  // Our rotation target
  public static Rotation2d targetHeading;
  public static final double ROTATION_DEGREES_TOLERANCE = 3;
  public static final double PIVOT_DEGREES_TOLERANCE = 0.5; // Fixme: constify

  public static final double pivoterOffset = 11;
  public static final double targetHeight = 70;

  // PIVOT Calculations: (UNUSED)
  // Lookup table for doubles
  double[][] pivoterLookUpTable = {
    {1, 2}, // Fixme: update values. 
    {1.25, 3.8},
    {1.5, 4.7},
    {1.7, 5.88},
    {2, 7},
    {2.25, 7.88},
    {2.5, 8.36}
  };

  // Track our pivoter target (UNUSED)
  double targetPivoterRotations;

  public SwerveAimAndPivot() {
    rotationController.setTolerance(Units.degreesToRadians(ROTATION_DEGREES_TOLERANCE));
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    // Get the apriltag position. 
    photonCamera = photonSub.getAprilCamera();
    // visionTab.addString("CameraTo Target", this::getFomattedTransform3d).withPosition(0, 0).withSize(2, 2);
    addRequirements(swerveSubsystem);
    addRequirements(photonSub);
    addRequirements(pivoterSub);
  }

  @Override
  public void initialize() {
    targetHeading = new Rotation2d(0);
    targetPivoterRotations = -6; // Value to which the pivoter will not rotate down to. 
   
    // Give our current swerve rotation. 
    rotationController.reset(swerveSubsystem.getRotation2d().getRadians());

    // Initialize our timer to start counting time aiming. 
    aimTimer.reset();
    aimTimer.start();
  }

  @Override
  public void execute() {
    var photonRes = photonCamera.getLatestResult();
    SmartDashboard.putBoolean("PhotonHasTarget", photonRes.hasTargets()); 


    if (photonRes.hasTargets()) {
      // Find the tag straight below speaker
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == RED_SPEAKER_TAG || t.getFiducialId() == BLUE_SPEAKER_TAG)
          .filter(t -> t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
          .findFirst();
    

      if (targetOpt.isPresent()) {
        photonCamera.setLED(VisionLEDMode.kBlink);
        var target = targetOpt.get();
        camToTarget = target.getBestCameraToTarget();
        ledSub.setBlue();
    
        // ----- ROTATION COMMAND ----- //
        // Calculate how far off we are rotation-wise from facing the apriltag's center. 
        var rotationDegrees = getRotationDegreesToSpeaker();
        if (rotationDegrees != 0.0) {
          // Get where the swerve is heading
          var drivetrainHeading = swerveSubsystem.getRotation2d();
          
          // Update our target heading - calculate how much the robot needs to rotate to center at the apriltag.
          targetHeading = drivetrainHeading.minus(Rotation2d.fromDegrees(rotationDegrees));
          rotateToSpeaker(targetHeading, drivetrainHeading); 
        }

        // Auto Pivot Mothod. 
        double tagDistance = PhotonUtils.calculateDistanceToTargetMeters(
          CAMERA_TO_ROBOT_X, 
          Units.inchesToMeters(59.65), 
          Math.PI / 6, 
          Units.degreesToRadians(targetOpt.get().getPitch()));

        // double lookUpVal = getPivoterOutputTable(tagDistance);
        
        // USE AN EQUATION TO GET VALUES
        double equationVal = (-6.5 + (9.9 * tagDistance) + (-1.63 * Math.pow(tagDistance, 2)));
        double equationValTwo = (-4.63 + (7.59 * tagDistance) + (-0.98 * Math.pow(tagDistance, 2)));

        double benEquationVal = (0.2875) * (157 - pivoterOffset - (Units.radiansToDegrees(Math.atan(targetHeight / (Units.metersToInches(tagDistance)))) + Units.radiansToDegrees(Math.acos((19 * Math.cos(Units.degreesToRadians(50))) / Math.sqrt((Math.pow(Units.metersToInches(tagDistance), 2)) + Math.pow(targetHeight, 2))))));
        // Ben spent weeks of his life on this. Please work pivoter gods. 
        //Constants.PivoterConstants.kPivoterGearRatio * 36
        System.out.println(equationVal);

        SmartDashboard.putNumber("Ben Equation Pivoter Value", benEquationVal);
        SmartDashboard.putNumber("Equation Pivoter Value", equationVal);
        SmartDashboard.putNumber("New Equation Pivoter Value", equationValTwo);

        if(tagDistance > 3.5) {
          targetPivoterRotations = benEquationVal;
        }
        else {
          targetPivoterRotations = equationValTwo;
        }
        
        // Check if we're within bounds
        if((targetPivoterRotations > PivoterConstants.kSubwofferPivoterRotations) && (targetPivoterRotations < PivoterConstants.kMaxPivoterRotations) ){

          System.out.print("Running at: ");
          System.out.println(targetPivoterRotations);
          // pivoterSub.PIDPivot(targetPivoterRotations);
          pivoterSub.PIDPivot(targetPivoterRotations);
          
        }else{
          System.out.print("NO WORK");
          System.out.println(targetPivoterRotations);
          
        }    
        // pivoterSub.PIDPivot(equationVal); // Fixme: Why do we call it twice?
        SmartDashboard.putNumber("Tag Pitch", targetOpt.get().getPitch());    
        SmartDashboard.putNumber("Apriltag Distance", tagDistance);    
        // SmartDashboard.putNumber("LookUpVal", lookUpVal);   
        SmartDashboard.putNumber("Target Pivot Rotations", targetPivoterRotations);   
      }

    } else {
      // If no target found, see if we have a previous angle we can use as a reference. 
      if (targetHeading.getRadians() != 0){
        ledSub.setYellow();

        // Rotate using our previous target
        rotateToSpeaker(targetHeading, swerveSubsystem.getRotation2d()); 
    
      } else {
        photonCamera.setLED(VisionLEDMode.kOff);
        System.out.print("NO REF FOUND");
        System.out.println(targetPivoterRotations);
        // Reset previous angle to be undefined
        swerveSubsystem.stopModules();
        ledSub.setRed();
        camToTarget = null;
      }
    }
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



  /*----- Pivot To Speaker and Rotate To Speaker Helper Functions ------- */


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


  // Return the degree the camera makes with the apriltag - how much we're off from being dead center to the apriltag. 
  private double getRotationDegreesToSpeaker() {
    return Math.atan2(
      camToTarget.getY() + CAMERA_TO_ROBOT_Y,
      camToTarget.getX() + CAMERA_TO_ROBOT_X)
      * (180 / Math.PI);
  }


  private String getFomattedTransform3d() {
    var transform3d = camToTarget;
    return transform3d == null ? "" : String.format("(%.2f, %.2f, %.2f)",
        transform3d.getX(),
        transform3d.getY(),
        transform3d.getZ());
  }


  //  private double getPivoterOutputTable(double distance){
  //   /* 1. go through each value in distance table. 
  //    * 2. Find the two indexes whose distances are inbetween the two. 
  //    * 3. Interpolate between the two to find the resulting distance.
  //   */
  //   for(int i = 0; i < pivoterLookUpTable.length - 1; i++){
  //     if((pivoterLookUpTable[i][0] < distance) && (distance < pivoterLookUpTable[i + 1][0])){
 
  //       // Get our pivoter values
  //       double lowerDistanceVal = pivoterLookUpTable[i][0];
  //       double lowerPivoterVal = pivoterLookUpTable[i][1];
        
  //       double higherDistanceVal = pivoterLookUpTable[i + 1][0];
  //       double higherPivoterVal = pivoterLookUpTable[i + 1][1];

  //       double slope = (higherPivoterVal - lowerPivoterVal) / (higherDistanceVal - lowerDistanceVal);
  //       double targetPivotVal = lowerPivoterVal + (distance - lowerDistanceVal) * (slope);
       
  //       SmartDashboard.putNumber("Lower PivoterVal", lowerPivoterVal);  
  //       SmartDashboard.putNumber("Slope", slope);    
  //       SmartDashboard.putNumber("Lower Distance", lowerDistanceVal); 
  //       SmartDashboard.putNumber("Higher Distance", higherDistanceVal); 
  //       SmartDashboard.putNumber("Target", targetPivotVal);   

  //       return targetPivotVal;
  //     }
  //   }
    
  //   System.out.println(" | NO TARGET FOUND");
  //   return -6; // This is a value that doesn't cause any rotation. 
  // }


  // private double getPivotDegreesToSpeaker() {
  //   return Math.atan2(
  //     camToTarget.getZ() + CAMERA_TO_ROBOT_Z + TAG_TO_SPEAKER_Z,
  //     Math.sqrt(Math.pow(camToTarget.getX() + CAMERA_TO_ROBOT_X, 2) + Math.pow(camToTarget.getY() + CAMERA_TO_ROBOT_Y, 2)))
  //     * 180 / Math.PI;
  // }

  // private void pivotToSpeaker(){
  //   var pivotDegrees = getPivotDegreesToSpeaker();
  //   var pivotRotations = pivoterSub.convertDegreesToMotorRotations(pivotDegrees);

  //   if (pivotRotations < PivoterConstants.kMaxPivoterRotations && pivotRotations > 0) {
  //     // print()
  //     photonCamera.setLED(VisionLEDMode.kBlink);
  //   } else {
  //     photonCamera.setLED(VisionLEDMode.kOff);
  //   }
  //   // SmartDashboard.putNumber("Speaker Rotations", Units.degreesToRadians(rotationDegrees));
  // }

}