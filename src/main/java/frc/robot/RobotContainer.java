// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TragConstants;
import frc.robot.autos.BlueAlliance.Blue3AmpAuto;
import frc.robot.autos.BlueAlliance.Blue3AmpHailMary;
import frc.robot.autos.BlueAlliance.Blue3Speaker;
import frc.robot.autos.RedAlliance.Red3AmpAuto;
import frc.robot.autos.RedAlliance.Red3AmpHailMary;
import frc.robot.autos.RedAlliance.Red3Speaker;
import frc.robot.commands.ShooterIntake.IntakeFromGround;
import frc.robot.commands.ShooterIntake.ReverseIntake;
import frc.robot.commands.ShooterIntake.ShootAmp;
import frc.robot.commands.ShooterIntake.ShootManual;
import frc.robot.commands.SwerveJoystickCMD;
import frc.robot.commands.Pivoter.ManualPivotDown;
import frc.robot.commands.Pivoter.ManualPivotUp;
import frc.robot.commands.Pivoter.PIDManual;
import frc.robot.commands.Pivoter.PIDPivotToPosition;
import frc.robot.commands.Elevator.LowerBothElevators;
import frc.robot.commands.Elevator.LowerLeftElevator;
import frc.robot.commands.Elevator.LowerRightElevator;
import frc.robot.commands.Elevator.RaiseBothElevators;
import frc.robot.commands.Elevator.RaiseLeftElevator;
import frc.robot.commands.Elevator.RaiseRightElevator;
import frc.robot.commands.Leds.LedFlash;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class RobotContainer {
  private final SendableChooser<String> commandChooser = new SendableChooser<>();
  public SequentialCommandGroup selectedAuto;
  private final XboxController xbox = Constants.OperatorConstants.xbox;

  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final LEDSubsystem ledSub = LEDSubsystem.getInstance();
  private final InstantCommand resetGyro = new InstantCommand(swerveSubsystem::zeroHeading, swerveSubsystem);
  
    private final SwerveJoystickCMD swerveCMD = new SwerveJoystickCMD(
                () -> -xbox.getRawAxis(OIConstants.kDriverYAxis),
                () -> xbox.getRawAxis(OIConstants.kDriverXAxis),
                () -> xbox.getRawAxis(OIConstants.kDriverRotAxis),
                () -> true/*
                () -> !xbox.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx */);

  public RobotContainer() {

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

    // Have our default swerve command to be the one that allows us to drive it. 
    CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, swerveCMD);
    configureBindings();
  }

  private void configureBindings() {
    // Driver commands for resetting the heading or position
    Constants.OperatorConstants.buttonX.onTrue(resetGyro);
    // Constants.OperatorConstants.buttonY.onTrue(new InstantCommand(() -> ledSub.setPurple()));
    // Indicate that we want to boost
    Constants.OperatorConstants.buttonY.onTrue(new LedFlash());

    // Commands regarding the intake sandwich  and Elevator
    // Constants.OperatorConstants.button1.onTrue(new ShooterVelocityPID(5000));
    // Constants.OperatorConstants.button1.onTrue(new ShootSpeaker());
    Constants.OperatorConstants.button1.whileTrue(new ShootManual());
    Constants.OperatorConstants.button2.whileTrue(new ShootAmp());
    Constants.OperatorConstants.button3.whileTrue(new LowerBothElevators());
    Constants.OperatorConstants.button4.whileTrue(new IntakeFromGround());
    Constants.OperatorConstants.button5.whileTrue(new RaiseBothElevators());
    Constants.OperatorConstants.button6.whileTrue(new ReverseIntake());   

    // Commands for the pivoter ARGH!! (╯°□°)╯︵ ┻━┻
    Constants.OperatorConstants.button7.onTrue(new PIDPivotToPosition(0));
    Constants.OperatorConstants.button8.onTrue(new PIDPivotToPosition(4.97777777983)); // Subwoofer Angle
    Constants.OperatorConstants.button9.onTrue(new PIDPivotToPosition(28.5)); // Amp Angle
    Constants.OperatorConstants.button10.onTrue(new PIDPivotToPosition(20.5)); // Stage Shot
    Constants.OperatorConstants.button11.onTrue(new PIDManual(false).andThen(new WaitCommand(0.5))); // Manual Down
    Constants.OperatorConstants.button12.onTrue(new PIDManual(true).andThen(new WaitCommand(0.5))); // Manual Up

    // Commands for elevator hahahah lmao
    Constants.OperatorConstants.pancakeUp.whileTrue(new RaiseLeftElevator());
    Constants.OperatorConstants.pancakeDown.whileTrue(new LowerLeftElevator());
    Constants.OperatorConstants.pancakeRight.whileTrue(new RaiseRightElevator());
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

    SwerveControllerCommand originToFarCenterNote = new SwerveControllerCommand(
      TragConstants.tragOriginToFarCenterNote, 
      swerveSubsystem::getPose,
      DriveConstants.kDriveKinematics,
      AutoConstants.xController,
      AutoConstants.yController, 
      AutoConstants.thetaController, 
      swerveSubsystem::setModuleStates,
      swerveSubsystem);

    // Chooser selection 
    if (commandChooser.getSelected() == "R3A"){
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

    // Return our selected auto to be run. 
    return selectedAuto;
  }
}
