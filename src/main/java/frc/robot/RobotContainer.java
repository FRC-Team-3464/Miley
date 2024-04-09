// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivoterConstants;
import frc.robot.Constants.SandwichConstants;
import frc.robot.commands.ShooterIntake.IntakeFromGround;
import frc.robot.commands.ShooterIntake.ReverseIntake;
import frc.robot.commands.ShooterIntake.RunIntake;
import frc.robot.commands.ShooterIntake.ShootAmp;
import frc.robot.commands.ShooterIntake.ShootManual;
import frc.robot.commands.ShooterIntake.AutoShootPID;
import frc.robot.commands.ShooterIntake.ShooterVelocityPID;
import frc.robot.commands.Swerve.SwerveAimAndPivot;
import frc.robot.commands.Swerve.SwerveAimNote;
import frc.robot.commands.Swerve.SwerveJoystickCMD;
import frc.robot.commands.Pivoter.PIDManual;
import frc.robot.commands.Pivoter.PIDPivotToPosition;
import frc.robot.commands.Pivoter.PIDPivotToZero;
import frc.robot.commands.Pivoter.PivotAmpAndShoot;
import frc.robot.commands.Elevator.LowerBothElevators;
import frc.robot.commands.Elevator.LowerLeftElevator;
import frc.robot.commands.Elevator.LowerRightElevator;
import frc.robot.commands.Elevator.RaiseBothElevators;
import frc.robot.subsystems.SwerveSubsystem;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class RobotContainer {
  public SequentialCommandGroup selectedAuto;
  private final XboxController xbox = Constants.OperatorConstants.xbox;

  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  // private final LEDSubsystem ledSub = LEDSubsystem.getInstance();
  private final InstantCommand resetGyro = new InstantCommand(swerveSubsystem::zeroHeading, swerveSubsystem);

  // Build an auto chooser. This will use Commands.none() as the default option.
  SendableChooser<Command> autoChooser;

  
    private final SwerveJoystickCMD swerveCMD = new SwerveJoystickCMD(
                () -> -xbox.getRawAxis(OIConstants.kDriverYAxis),
                () -> xbox.getRawAxis(OIConstants.kDriverXAxis),
                () -> xbox.getRawAxis(OIConstants.kDriverRotAxis),
                () -> true);
                // () -> !xbox.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx));

  public RobotContainer() {
    CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, swerveCMD);

    /* --------------------- PATHPLANNER Named Commands: Commands We'll Use During Auto--------------------- */ 
    NamedCommands.registerCommand("Pivot to Subwoofer", new PIDPivotToPosition(PivoterConstants.kSubwofferPivoterRotations));
    NamedCommands.registerCommand("Pivot to Ground", new PIDPivotToPosition(0));
    NamedCommands.registerCommand("Pivot to Amp", new PIDPivotToPosition(PivoterConstants.kPostAmpPivoterRotations));
    NamedCommands.registerCommand("Pivot to Stage", new PIDPivotToPosition(PivoterConstants.kStagePivoterRotations));
    NamedCommands.registerCommand("Pivot to Amp-Stage", new PIDPivotToPosition(PivoterConstants.kStagePivoterRotations));
    NamedCommands.registerCommand("Force Pivot to Ground", new PIDPivotToZero());
    
    NamedCommands.registerCommand("Shoot Speaker", new ShootManual());
    NamedCommands.registerCommand("Shoot Amp", new ShootAmp());
    NamedCommands.registerCommand("Trigger Intake", new RunIntake(SandwichConstants.kTriggerIntakeSpeed)); 

    NamedCommands.registerCommand("Shoot PID Speaker", new AutoShootPID());
    NamedCommands.registerCommand("Reverse Intake", new ReverseIntake());
    NamedCommands.registerCommand("Start Shooter", new ShooterVelocityPID(SandwichConstants.kShootVelocityTarget));
    NamedCommands.registerCommand("Stop Shooter", new ShooterVelocityPID(0));

    NamedCommands.registerCommand("Intake From Ground", new IntakeFromGround());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    configureBindings();
  }

  private void configureBindings() {
    // Driver commands for resetting the heading or position
    Constants.OperatorConstants.buttonX.onTrue(resetGyro);
    // Constants.OperatorConstants.buttonY.whileTrue(new SwerveAimSpeaker());
    Constants.OperatorConstants.buttonLB.whileTrue(new SwerveAimNote());
    Constants.OperatorConstants.buttonRB.whileTrue(new SwerveAimAndPivot());
    // Indicate that we want to boost
    // Constants.OperatorConstants.button1.onTrue(new LedFlash());

    // Commands regarding the intake sandwich  and Elevator
    // Constants.OperatorConstants.button1.onTrue(new ShootPID());
    // Constants.OperatorConstants.button1.onTrue(new ShootSpeaker());
    // Constants.OperatorConstants.button1.whileTrue(new ShootManual());
    Constants.OperatorConstants.button1.onTrue(new ShooterVelocityPID(SandwichConstants.kShootVelocityTarget));
    Constants.OperatorConstants.button1.onFalse(new ShooterVelocityPID(0));
    Constants.OperatorConstants.button2.whileTrue(new PivotAmpAndShoot());
    Constants.OperatorConstants.button3.whileTrue(new LowerBothElevators());
    Constants.OperatorConstants.button4.whileTrue(new IntakeFromGround());
    Constants.OperatorConstants.button5.whileTrue(new RaiseBothElevators());
    Constants.OperatorConstants.button6.whileTrue(new ReverseIntake());   
    Constants.OperatorConstants.button12.onTrue(new PIDPivotToZero());

    // Intake Trigger
    Constants.OperatorConstants.button11.whileTrue(new RunIntake(SandwichConstants.kTriggerIntakeSpeed));

    // Constants.OperatorConstants.button3.onTrue(new InstantCommand(() -> intakeSub.runServo(0.5)));
    // Constants.OperatorConstants.button5.onTrue(new InstantCommand(() -> intakeSub.runServo(0)));

    // Commands for the pivoter ARGH!! (╯°□°)╯︵ ┻━┻
    Constants.OperatorConstants.button7.onTrue(new PIDPivotToPosition(0));
    Constants.OperatorConstants.button8.onTrue(new PIDPivotToPosition(Constants.PivoterConstants.kSubwofferPivoterRotations)); // Subwoofer Angle
    Constants.OperatorConstants.button9.onTrue(new PIDPivotToPosition(Constants.PivoterConstants.kPreAmpPivoterRotations)); // Amp Angle
    Constants.OperatorConstants.button10.onTrue(new PIDPivotToPosition(Constants.PivoterConstants.kStagePivoterRotations)); // Stage Shot
    Constants.OperatorConstants.pancakeUp.onTrue(new PIDManual(true).andThen(new WaitCommand(0.5))); 
    Constants.OperatorConstants.pancakeDown.onTrue(new PIDManual(false).andThen(new WaitCommand(0.5)));
   
    // Commands for elevator hahahah lmao
    // Constants.OperatorConstants.pancakeUp.whileTrue(new RaiseLeftElevator());
    // Constants.OperatorConstants.pancakeRight.whileTrue(new RaiseRightElevator());
    Constants.OperatorConstants.pancakeRight.whileTrue(new LowerLeftElevator());
    Constants.OperatorConstants.pancakeLeft.whileTrue(new LowerRightElevator()); 
  }
 

  public Command getAutonomousCommand() { 
    // Config our theta controller to calculate error in a circle.
    AutoConstants.thetaController.enableContinuousInput(-Math.PI, Math.PI);  

    // Get the auto selected from pathplanner
    return autoChooser.getSelected();
  }
}
