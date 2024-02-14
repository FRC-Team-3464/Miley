// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TragConstants;
import frc.robot.autos.BlueAlliance.Blue3AmpAuto;
import frc.robot.autos.BlueAlliance.Blue3AmpHailMary;
import frc.robot.autos.BlueAlliance.Blue3Speaker;
import frc.robot.autos.RedAlliance.Red3AmpAuto;
import frc.robot.autos.RedAlliance.Red3AmpHailMary;
import frc.robot.autos.RedAlliance.Red3Speaker;
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
import frc.robot.trajectories.AmpTrajectories;
import frc.robot.trajectories.SpeakerTrajectories;
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

  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final InstantCommand resetGyro = new InstantCommand(swerveSubsystem::zeroHeading, swerveSubsystem);
  // private final InstantCommand resetLocation = new InstantCommand(swerveSubsystem::resetOdom)
    private final SwerveJoystickCMD swerveCMD = new SwerveJoystickCMD(
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
    Constants.OperatorConstants.button2.whileTrue(new ReverseIntake());
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
 

    AutoConstants.thetaController.enableContinuousInput(-Math.PI, Math.PI);  

      /*
       * 
       * BLUE Autos
       * 
       */


    // ------------------- Points relative to speaker ----------------------- //

    /*
    * Blue Alliance SPEAKER
    */ 
    

      /*
       * 
       * Hail Mary Note
       * 
       */

      SwerveControllerCommand originToFarCenterNote = new SwerveControllerCommand(
        TragConstants.tragOriginToFarCenterNote, 
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        AutoConstants.xController,
        AutoConstants.yController, 
        AutoConstants.thetaController, 
        swerveSubsystem::setModuleStates,
        swerveSubsystem);




      /*
       * 
       * AMP Autos
       * 
       */
      

    // Start HERE:
    if (commandChooser.getSelected() == "R3A"){
      // add some init and wrap up, and return everything
      selectedAuto = new Red3AmpAuto();

    }else if (commandChooser.getSelected() == "R3AHM"){
      // HAIL MARY
      selectedAuto = new Red3AmpHailMary();
      
    }else if(commandChooser.getSelected() == "R3S"){
      selectedAuto = new Red3Speaker();

    }else if(commandChooser.getSelected() == "B3A"){
      selectedAuto = new Blue3AmpAuto();

    }else if (commandChooser.getSelected() == "B3AHM"){
      selectedAuto = new Blue3AmpHailMary();       

    }else if(commandChooser.getSelected() == "B3S"){
      selectedAuto = new Blue3Speaker();
    
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
