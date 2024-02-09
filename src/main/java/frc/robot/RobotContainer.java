// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TragConstants;
import frc.robot.commands.Intake;
import frc.robot.commands.LowerLeftElevator;
import frc.robot.commands.LowerRightElevator;
import frc.robot.commands.RaiseBothElevators;
import frc.robot.commands.RaiseLeftElevator;
import frc.robot.commands.RaiseRightElevator;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ShootAmp;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.SwerveJoystickCMD;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ShooterSubsystem shootSub;
  private final IntakeSubsystem intakeSub;
  private final ElevatorSubsystem elevatorSub;

  private final Intake suck = new Intake();
  private final ReverseIntake spit = new ReverseIntake();

  private final ShootSpeaker speaker = new ShootSpeaker();
  private final ShootAmp amp = new ShootAmp();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<String> commandChooser = new SendableChooser<>();
  public SequentialCommandGroup selectedAuto;
  private final XboxController xbox = Constants.OperatorConstants.xbox;

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final InstantCommand resetGyro = new InstantCommand(swerveSubsystem::zeroHeading, swerveSubsystem);
  // private final InstantCommand resetLocation = new InstantCommand(swerveSubsystem::resetOdom)
    private final SwerveJoystickCMD swerveCMD = new SwerveJoystickCMD(swerveSubsystem,
                () -> -xbox.getRawAxis(OIConstants.kDriverYAxis),
                () -> xbox.getRawAxis(OIConstants.kDriverXAxis),
                () -> xbox.getRawAxis(OIConstants.kDriverRotAxis),
                () -> true/*
                () -> !xbox.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx */);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    elevatorSub = ElevatorSubsystem.getInstance();
    shootSub = ShooterSubsystem.getInstance();
    intakeSub = IntakeSubsystem.getInstance();

        // where we set the options that user has to choose for autos 
    // Red Autos
    commandChooser.setDefaultOption("Red 3 Amp ", "R3A");
    commandChooser.addOption("Red 3 Amp Hail Mary", "R3AHM");
    commandChooser.addOption("Red 3 Speaker", "R3S");
    commandChooser.addOption("Red Center Hail Mary", "RCHM");    
    // Blue Autos
    commandChooser.addOption("Blue 3 Amp", "B3A");
    commandChooser.addOption("Blue 3 Amp Hail Mary", "B3AHM");
    commandChooser.addOption("Blue 3 Speaker", "B3S");

    SmartDashboard.putData("Auto", commandChooser);
    CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, swerveCMD);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    Constants.OperatorConstants.buttonX.onTrue(resetGyro);
    Constants.OperatorConstants.buttonY.onTrue(new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)))));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    Constants.OperatorConstants.button1.whileTrue(new ShootSpeaker());
    Constants.OperatorConstants.button12.whileTrue(spit);
    Constants.OperatorConstants.button4.whileTrue(new Intake());

    Constants.OperatorConstants.pancakeUp.whileTrue(new RaiseLeftElevator());
    Constants.OperatorConstants.pancakeDown.whileTrue(new LowerLeftElevator());
    Constants.OperatorConstants.pancakeRight.whileTrue(new RaiseRightElevator());
    Constants.OperatorConstants.pancakeLeft.whileTrue(new LowerRightElevator());
    Constants.OperatorConstants.button5.whileTrue(new RaiseBothElevators());
    Constants.OperatorConstants.button3.whileTrue(new LowerBothElevators());
  }
 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);

      // Profiled PID Controller = PID Controller with constraints on max speed / acceleration. 
      ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController,
        0,
        0,
        AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);  

      /*
       * 
       * AMP Autos
       * 
       */

      // contruct command to follow trajectory
      SwerveControllerCommand orginToAmp = new SwerveControllerCommand(
        TragConstants.tragOriginToAmp, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand ampToSpeaker = new SwerveControllerCommand(
        TragConstants.tragAmpToSpeakerNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);


      SwerveControllerCommand ampToAmpN = new SwerveControllerCommand(
        TragConstants.tragAmpToAmpNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand ampNToAmp = new SwerveControllerCommand(
        TragConstants.tragAmpNoteToAmp, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);
      
      SwerveControllerCommand speakerNoteToAmp = new SwerveControllerCommand(
        TragConstants.tragSpeakerNoteToAmp, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand ampToRightmostNote = new SwerveControllerCommand(
        TragConstants.tragAmpToHailMaryNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand rightmostNoteToAmpM1 = new SwerveControllerCommand(
        TragConstants.tragHailMaryNoteToAmpM1, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      /*
       * 
       * BLUE Autos
       * 
       */

      // contruct command to follow trajectory
      SwerveControllerCommand blueOrginToAmp = new SwerveControllerCommand(
        TragConstants.tragBlueOriginToAmp, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand blueAmpToSpeaker = new SwerveControllerCommand(
        TragConstants.tragBlueAmpToSpeakerNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);


      SwerveControllerCommand blueAmpToAmpN = new SwerveControllerCommand(
        TragConstants.tragBlueAmpToAmpNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand blueAmpNToAmp = new SwerveControllerCommand(
        TragConstants.tragBlueAmpNoteToAmp, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);
      
      SwerveControllerCommand blueSpeakerNoteToAmp = new SwerveControllerCommand(
        TragConstants.tragBlueSpeakerNoteToAmp, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand blueAmpToRightmostNote = new SwerveControllerCommand(
        TragConstants.tragBlueAmpToHailMaryNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand blueRightmostNoteToAmpM1 = new SwerveControllerCommand(
        TragConstants.tragBlueHailMaryNoteToAmpM1, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);



    // ------------------- Points relative to speaker ----------------------- //

    SwerveControllerCommand originToStageNote = new SwerveControllerCommand(
        TragConstants.tragOriginToStageNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);
        
    SwerveControllerCommand stageNoteToSpeakerShooting = new SwerveControllerCommand(
        TragConstants.tragStageNoteToSpeakerShooting, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand speakerShootingToSpeakerNote = new SwerveControllerCommand(
        TragConstants.tragSpeakerShootingToSpeakerNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand speakerNoteToAmpShooting = new SwerveControllerCommand(
        TragConstants.tragSpeakerNoteToAmpShooting, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);
      
      SwerveControllerCommand ampShootingToAmpNote = new SwerveControllerCommand(
        TragConstants.tragAmpShootingToAmpNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);
    
      SwerveControllerCommand ampNoteRotateToSpeaker = new SwerveControllerCommand(
        TragConstants.tragAmpNoteRotateToSpeaker, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);
    
    /*
    * Blue Alliance SPEAKER
    */ 
    
    SwerveControllerCommand blueOriginToStageNote = new SwerveControllerCommand(
        TragConstants.tragBlueOriginToStageNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);
        
    SwerveControllerCommand blueStageNoteToSpeakerShooting = new SwerveControllerCommand(
        TragConstants.tragBlueStageNoteToSpeakerShooting, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand blueSpeakerShootingToSpeakerNote = new SwerveControllerCommand(
        TragConstants.tragBlueSpeakerShootingToSpeakerNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand blueSpeakerNoteToAmpShooting = new SwerveControllerCommand(
        TragConstants.tragBlueSpeakerNoteToAmpShooting, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);
      
      SwerveControllerCommand blueAmpShootingToAmpNote = new SwerveControllerCommand(
        TragConstants.tragBlueAmpShootingToAmpNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);
    
      SwerveControllerCommand blueAmpNoteRotateToSpeaker = new SwerveControllerCommand(
        TragConstants.tragBlueAmpNoteRotateToSpeaker, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      /*
       * 
       * Hail Mary Note
       * 
       */

      SwerveControllerCommand originToFarCenterNote = new SwerveControllerCommand(
        TragConstants.tragOriginToFarCenterNote, 
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController, 
        thetaController, 
        swerveSubsystem::setModuleStates,
        swerveSubsystem);
      

    // Start HERE:
    if (commandChooser.getSelected() == "R3A"){
      // add some init and wrap up, and return everything
      selectedAuto = new SequentialCommandGroup(
        // Reset odometry to starting pose. 
        new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragOriginToAmp.getInitialPose())),
        orginToAmp,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragAmpToAmpNote.getInitialPose())),
        ampToAmpN,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragAmpNoteToAmp.getInitialPose())),
        ampNToAmp,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragAmpToSpeakerNote.getInitialPose())),
        ampToSpeaker,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragSpeakerNoteToAmp.getInitialPose())),
        speakerNoteToAmp,
        new InstantCommand(() -> swerveSubsystem.stopModules()));

    }else if (commandChooser.getSelected() == "R3AHM"){
        // HAIL MARY
            // add some init and wrap up, and return everything
        selectedAuto = new SequentialCommandGroup(
          // Reset odometry to starting pose. 
          new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragOriginToAmp.getInitialPose())),
          orginToAmp,
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
          new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragAmpToAmpNote.getInitialPose())),
          ampToAmpN,
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
          new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragAmpNoteToAmp.getInitialPose())),
          ampNToAmp,
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
          new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragAmpToHailMaryNote.getInitialPose())),
          ampToRightmostNote,
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
          new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragHailMaryNoteToAmpM1.getInitialPose())),
          new WaitCommand(0.25),
          rightmostNoteToAmpM1,
          new InstantCommand(() -> swerveSubsystem.stopModules())
        );
    }else if(commandChooser.getSelected() == "R3S"){
      selectedAuto = new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragOriginToStageNote.getInitialPose())),
        originToStageNote,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        // Go to Speaker Note
        new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragStageNoteToSpeakerShooting.getInitialPose())),
        stageNoteToSpeakerShooting,
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragSpeakerShootingToSpeakerNote.getInitialPose())),
        speakerShootingToSpeakerNote,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),

        // Go to Amp Note
        new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragSpeakerNoteToAmpShooting.getInitialPose())),
        speakerNoteToAmpShooting,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragAmpShootingToAmpNote.getInitialPose())),
        ampShootingToAmpNote,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        // Rotate to speaker
        new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragAmpNoteRotateToSpeaker.getInitialPose())),
        ampNoteRotateToSpeaker,
        new InstantCommand(() -> swerveSubsystem.stopModules())
        // new WaitCommand(0.25)
        );
    }else if(commandChooser.getSelected() == "B3A"){
      selectedAuto = new SequentialCommandGroup(
          // Reset odometry to starting pose. 
          new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueOriginToAmp.getInitialPose())),
          blueOrginToAmp,
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
          new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueAmpToAmpNote.getInitialPose())),
          blueAmpToAmpN,
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
          new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueAmpNoteToAmp.getInitialPose())),
          blueAmpNToAmp,
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
          new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueAmpToSpeakerNote.getInitialPose())),
          blueAmpToSpeaker,
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
          new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueSpeakerNoteToAmp.getInitialPose())),
          blueSpeakerNoteToAmp,
          new InstantCommand(() -> swerveSubsystem.stopModules())
          );  

    }else if (commandChooser.getSelected() == "B3AHM"){
      selectedAuto = new SequentialCommandGroup(
        // Reset odometry to starting pose. 
        new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueOriginToAmp.getInitialPose())),
        blueOrginToAmp,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueAmpToAmpNote.getInitialPose())),
        blueAmpToAmpN,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueAmpNoteToAmp.getInitialPose())),
        blueAmpNToAmp,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueAmpToHailMaryNote.getInitialPose())),
        blueAmpToRightmostNote,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueHailMaryNoteToAmpM1.getInitialPose())),
        new WaitCommand(0.25),
        blueRightmostNoteToAmpM1,
        new InstantCommand(() -> swerveSubsystem.stopModules())
      );       

      }else if(commandChooser.getSelected() == "B3S"){
        selectedAuto = new SequentialCommandGroup(
          new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueOriginToStageNote.getInitialPose())),
          blueOriginToStageNote,
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
          // Go to Speaker Note
          new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueStageNoteToSpeakerShooting.getInitialPose())),
          blueStageNoteToSpeakerShooting,
          new WaitCommand(0.25),
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
          new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueSpeakerShootingToSpeakerNote.getInitialPose())),
          blueSpeakerShootingToSpeakerNote,
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
  
          // Go to Amp Note
          new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueSpeakerNoteToAmpShooting.getInitialPose())),
          blueSpeakerNoteToAmpShooting,
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
          new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueAmpShootingToAmpNote.getInitialPose())),
          blueAmpShootingToAmpNote,
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
          // Rotate to speaker
          new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueAmpNoteRotateToSpeaker.getInitialPose())),
          blueAmpNoteRotateToSpeaker,
          new InstantCommand(() -> swerveSubsystem.stopModules())
          // new WaitCommand(0.25)
          );
      
      }else if(commandChooser.getSelected() == "RCHM"){
          selectedAuto = new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragOriginToFarCenterNote.getInitialPose())),
            originToFarCenterNote,
            new InstantCommand(() -> swerveSubsystem.stopModules())
            );


      } else{
        selectedAuto = null;
    }


    return selectedAuto;
  }
}
