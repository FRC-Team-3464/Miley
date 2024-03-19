// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivoterConstants;
import frc.robot.Constants.TragConstants;
import frc.robot.autos.BlueAlliance.Blue1AmpAuto;
import frc.robot.autos.BlueAlliance.Blue1Speaker;
import frc.robot.autos.BlueAlliance.Blue2AmpAuto;
import frc.robot.autos.BlueAlliance.Blue3AmpHailMary;
import frc.robot.autos.BlueAlliance.Blue2Speaker;
import frc.robot.autos.BlueAlliance.Blue2StraightSpeaker;
import frc.robot.autos.RedAlliance.Red1AmpAuto;
import frc.robot.autos.RedAlliance.Red1Speaker;
import frc.robot.autos.RedAlliance.Red2AmpAuto;
import frc.robot.autos.RedAlliance.Red3AmpHailMary;
import frc.robot.autos.RedAlliance.Red2Speaker;
import frc.robot.autos.RedAlliance.Red2StraightSpeaker;
import frc.robot.commands.ShooterIntake.IntakeFromGround;
import frc.robot.commands.ShooterIntake.ManualIntake;
import frc.robot.commands.ShooterIntake.ReverseIntake;
import frc.robot.commands.ShooterIntake.RunIntake;
import frc.robot.commands.ShooterIntake.ShootAmp;
import frc.robot.commands.ShooterIntake.ShootManual;
import frc.robot.commands.ShooterIntake.ShooterVelocityPID;
// import frc.robot.commands.ShooterIntake.ShootSpeaker;
// import frc.robot.commands.ShooterIntake.ShootManual;
import frc.robot.commands.SwerveJoystickCMD;
import frc.robot.commands.Pivoter.ManualPivotUp;
// import frc.robot.commands.Pivoter.ManualPivotDown;
// import frc.robot.commands.Pivoter.ManualPivotUp;
import frc.robot.commands.Pivoter.PIDManual;
import frc.robot.commands.Pivoter.PIDPivotToPosition;
import frc.robot.commands.Elevator.LowerBothElevators;
import frc.robot.commands.Elevator.LowerLeftElevator;
import frc.robot.commands.Elevator.LowerRightElevator;
import frc.robot.commands.Elevator.RaiseBothElevators;
import frc.robot.commands.Elevator.RaiseLeftElevator;
import frc.robot.commands.Elevator.RaiseRightElevator;
import frc.robot.commands.Leds.LedFlash;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivoterSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class RobotContainer {
  private final SendableChooser<String> commandChooser = new SendableChooser<>();
  public SequentialCommandGroup selectedAuto;
  private final XboxController xbox = Constants.OperatorConstants.xbox;

  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final PivoterSubsystem pivotSub = PivoterSubsystem.getInstance();
  private final IntakeSubsystem intakeSub = IntakeSubsystem.getInstance();
  // private final LEDSubsystem ledSub = LEDSubsystem.getInstance();
  private final InstantCommand resetGyro = new InstantCommand(swerveSubsystem::zeroHeading, swerveSubsystem);

  // Build an auto chooser. This will use Commands.none() as the default option.
  SendableChooser<Command> autoChooser;

  
    private final SwerveJoystickCMD swerveCMD = new SwerveJoystickCMD(
                () -> -xbox.getRawAxis(OIConstants.kDriverYAxis),
                () -> xbox.getRawAxis(OIConstants.kDriverXAxis),
                () -> xbox.getRawAxis(OIConstants.kDriverRotAxis),
                // () -> true/*
                () -> !xbox.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx));

  public RobotContainer() {

    // // where we set the options that user has to choose for autos 
    // // Red Autos
    // commandChooser.addOption("Red 1 Amp ", "R1A");
    // commandChooser.addOption("Red 2 Amp ", "R2A");
    // // commandChooser.addOption("Red 3 Amp Hail Mary", "R3AHM");
    // commandChooser.addOption("Red 1 Speaker ", "R1S");
    // commandChooser.setDefaultOption("Red 2 Speaker", "R2S");
    // commandChooser.setDefaultOption("Red 2 Straight Speaker", "R2SS");
    // commandChooser.addOption("Red Center Hail Mary", "RCHM");    
    // // Blue Autos
    // commandChooser.addOption("Blue 1 Amp", "B1A");
    // commandChooser.addOption("Blue 2 Amp", "B2A");
    // // commandChooser.addOption("Blue 3 Amp Hail Mary", "B3AHM");
    // commandChooser.addOption("Blue 1 Speaker ", "B1S");
    // commandChooser.addOption("Blue 2 Speaker", "B2S");
    // commandChooser.setDefaultOption("Blue 2 Straight Speaker", "B2SS");
    // commandChooser.addOption("Blue Center Hail Mary", "BCHM");    
    
    // SmartDashboard.putData("Auto", commandChooser);

    // // Have our default swerve command to be the one that allows us to drive it. 
    CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, swerveCMD);

    /* --------------------- PATHPLANNER --------------------- */ 
    NamedCommands.registerCommand("Pivot to Subwoofer", new PIDPivotToPosition(Constants.PivoterConstants.kSubwofferPivoterRotations));
    NamedCommands.registerCommand("Pivot to Ground", new PIDPivotToPosition(0));
    NamedCommands.registerCommand("Pivot to Amp", new PIDPivotToPosition(Constants.PivoterConstants.kAmpPivoterRotations));
    
    NamedCommands.registerCommand("Shoot Speaker", new ShootManual());
    NamedCommands.registerCommand("Shoot Amp", new ShootAmp());
    NamedCommands.registerCommand("Trigger Intake", new RunIntake(Constants.SandwichConstants.kIntakeSpeed));
    NamedCommands.registerCommand("Reverse Intake", new ReverseIntake());
    
    NamedCommands.registerCommand("Start Shooter", new ShooterVelocityPID(4000));
    NamedCommands.registerCommand("Stop Shooter", new ShooterVelocityPID(0));

    NamedCommands.registerCommand("Intake From Ground", new IntakeFromGround());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    configureBindings();
  }

  private void configureBindings() {
    // Driver commands for resetting the heading or position
    Constants.OperatorConstants.buttonX.onTrue(resetGyro);
    // Constants.OperatorConstants.buttonY.onTrue(new InstantCommand(() -> ledSub.setPurple()));
    // Indicate that we want to boost
    Constants.OperatorConstants.buttonY.onTrue(new LedFlash());

    // Commands regarding the intake sandwich  and Elevator
    // Constants.OperatorConstants.button1.onTrue(new ShooterVelocityPID(4000));
    // Constants.OperatorConstants.button1.onTrue(new ShootSpeaker());
    Constants.OperatorConstants.button1.onTrue(new ShooterVelocityPID(4000));
    Constants.OperatorConstants.button1.onFalse(new ShooterVelocityPID(0));
    // Constants.OperatorConstants.button1.whileTrue(new ShootManual());
    Constants.OperatorConstants.button2.whileTrue(new ShootAmp());
    // Constants.OperatorConstants.button3.whileTrue(new LowerBothElevators());
    Constants.OperatorConstants.button4.whileTrue(new IntakeFromGround());
    // Constants.OperatorConstants.button5.whileTrue(new RaiseBothElevators());
    Constants.OperatorConstants.button6.whileTrue(new ReverseIntake());   
    Constants.OperatorConstants.button12.whileTrue(new ManualIntake());
    Constants.OperatorConstants.button11.onTrue(new RunIntake(0.8));


    Constants.OperatorConstants.button3.onTrue(new InstantCommand(() -> intakeSub.runServo(0.5)));
    Constants.OperatorConstants.button5.onTrue(new InstantCommand(() -> intakeSub.runServo(0)));

    // Commands for the pivoter ARGH!! (╯°□°)╯︵ ┻━┻
    Constants.OperatorConstants.button7.onTrue(new PIDPivotToPosition(0));
    Constants.OperatorConstants.button8.onTrue(new PIDPivotToPosition(Constants.PivoterConstants.kSubwofferPivoterRotations)); // Subwoofer Angle
    Constants.OperatorConstants.button9.onTrue(new PIDPivotToPosition(Constants.PivoterConstants.kAmpPivoterRotations)); // Amp Angle
    Constants.OperatorConstants.button10.onTrue(new PIDPivotToPosition(Constants.PivoterConstants.kStagePivoterRotations)); // Stage Shot
    // Constants.OperatorConstants.button11.onTrue(new PIDManual(false).andThen(new WaitCommand(0.5))); // Manual Down
    Constants.OperatorConstants.pancakeUp.onTrue(new PIDManual(true).andThen(new WaitCommand(0.5))); 
    Constants.OperatorConstants.pancakeDown.onTrue(new PIDManual(false).andThen(new WaitCommand(0.5)));
    // Constants.OperatorConstants.pancakeUp.whileTrue(new ManualPivotUp());
    // Constants.OperatorConstants.button12.onTrue(new PIDManual(true).andThen(new WaitCommand(0.5))); // Manual Up

    // Commands for elevator hahahah lmao
    // Constants.OperatorConstants.pancakeUp.whileTrue(new RaiseLeftElevator());
    Constants.OperatorConstants.pancakeRight.whileTrue(new LowerLeftElevator());
    // Constants.OperatorConstants.pancakeRight.whileTrue(new RaiseRightElevator());
    Constants.OperatorConstants.pancakeLeft.whileTrue(new LowerRightElevator());

    // Test positions
    // NON-PID
    // Constants.OperatorConstants.button7.onTrue(new PivotToPosition(0));
    // Constants.OperatorConstants.button8.onTrue(new PivotToPosition(20));
    // Constants.OperatorConstants.button9.onTrue(new PivotToPosition(100)); 
  }
 

  public Command getAutonomousCommand() { 
    // Config our theta controller to calculate error in a circle.
    AutoConstants.thetaController.enableContinuousInput(-Math.PI, Math.PI);  

    // SwerveControllerCommand originToFarCenterNote = new SwerveControllerCommand(
    //   TragConstants.tragOriginToFarCenterNote, 
    //   swerveSubsystem::getPose,
    //   DriveConstants.kDriveKinematics,
    //   AutoConstants.xController,
    //   AutoConstants.yController, 
    //   AutoConstants.thetaController, 
    //   swerveSubsystem::setModuleStates,
    //   swerveSubsystem);

    // SwerveControllerCommand blueOriginToFarCenterNote = new SwerveControllerCommand(
    //   TragConstants.tragBlueOriginToFarCenterNote, 
    //   swerveSubsystem::getPose,
    //   DriveConstants.kDriveKinematics,
    //   AutoConstants.xController,
    //   AutoConstants.yController, 
    //   AutoConstants.thetaController, 
    //   swerveSubsystem::setModuleStates,
    //   swerveSubsystem);

    // // Chooser selection 
    // if (commandChooser.getSelected() == "R1A"){
    //   selectedAuto = new Red1AmpAuto();
      
    // }else if (commandChooser.getSelected() == "R2A"){
    //   selectedAuto = new Red2AmpAuto();

    // }else if (commandChooser.getSelected() == "R3AHM"){
    //   /* NOT USED FOR HARTFORD */
    //   // HAIL MARY
    //   selectedAuto = new Red3AmpHailMary();
      
    // }else if(commandChooser.getSelected() == "R1S"){
    //   selectedAuto = new Red1Speaker();

    // }else if(commandChooser.getSelected() == "R2S"){
    //   selectedAuto = new Red2Speaker();

    // }else if(commandChooser.getSelected() == "R2SS"){
    //   selectedAuto = new Red2StraightSpeaker();

    // }else if (commandChooser.getSelected() == "B1A"){
    //   selectedAuto = new Blue1AmpAuto();
      
    // }else if(commandChooser.getSelected() == "B2A"){
    //   selectedAuto = new Blue2AmpAuto();

    // }else if (commandChooser.getSelected() == "B3AHM"){
    //   /* NOT USED FOR HARTFORD */
    //   selectedAuto = new Blue3AmpHailMary();       

    // }else if(commandChooser.getSelected() == "B1S"){
    //   selectedAuto = new Blue1Speaker();

    // }else if(commandChooser.getSelected() == "B2S"){
    //   selectedAuto = new Blue2Speaker();
    
    // }else if(commandChooser.getSelected() == "B2SS"){
    //   selectedAuto = new Blue2StraightSpeaker();
    
    // }else if(commandChooser.getSelected() == "RCHM"){
    //   selectedAuto = new SequentialCommandGroup(
    //     // Shoot first into speaker
    //     new InstantCommand(() -> swerveSubsystem.stopModules()),
    //     new PIDPivotToPosition(PivoterConstants.kSubwofferPivoterRotations),
    //     new ParallelRaceGroup(
    //       new ShootManual(),
    //       new WaitCommand(2)        
    //     ),
    //     // Go to subwoffer pos;
    //     new PIDPivotToPosition(0),
    //     new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragOriginToFarCenterNote.getInitialPose())),
    //         // IF we reach a note and intake it - great. No need for fussy dual parallel commands
    //     new ParallelRaceGroup(
    //       new IntakeFromGround(),
    //       originToFarCenterNote
    //     ),

    //     new InstantCommand(() -> swerveSubsystem.stopModules())
    //     );

    // } else if(commandChooser.getSelected() == "BCHM"){
    //     selectedAuto = new SequentialCommandGroup(
    //       // Shoot first into speaker
    //       new InstantCommand(() -> swerveSubsystem.stopModules()),
    //       new PIDPivotToPosition(PivoterConstants.kSubwofferPivoterRotations),
    //       new ParallelRaceGroup(
    //        new ShootManual(),
    //        new WaitCommand(2)        
    //       ),
    //       // Go to subwoffer pos;
    //       new PIDPivotToPosition(0),
    //       new InstantCommand(() -> swerveSubsystem.resetOdometry(TragConstants.tragBlueOriginToFarCenterNote.getInitialPose())),
          
    //       new ParallelRaceGroup(
    //         new IntakeFromGround(),
    //         blueOriginToFarCenterNote

    //       ),
    //       new InstantCommand(() -> swerveSubsystem.stopModules())
    //       );

    // } else{
    //   selectedAuto = null;
    // }

    // // Return our selected auto to be run. 
    // return selectedAuto;
    return autoChooser.getSelected();
  }
}
