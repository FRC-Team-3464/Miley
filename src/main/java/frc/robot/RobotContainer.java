// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Intake;
import frc.robot.commands.RaiseLeftElevator;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ShootAmp;
import frc.robot.commands.ShootSpeaker;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final ElevatorSubsystem elevatorSub;
  private final ShooterSubsystem shootSub = new ShooterSubsystem();
  private final IntakeSubsystem intakeSub = new IntakeSubsystem();

  private final Intake suck = new Intake(intakeSub);
  private final ReverseIntake spit = new ReverseIntake(intakeSub);

  private final ShootSpeaker speaker = new ShootSpeaker(shootSub,intakeSub);
  private final ShootAmp amp = new ShootAmp(shootSub, intakeSub);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    elevatorSub = ElevatorSubsystem.getInstance();
    // Configure the trigger bindings
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    Constants.OperatorConstants.button1.whileTrue(speaker);
    Constants.OperatorConstants.button2.whileTrue(spit);
    Constants.OperatorConstants.button3.whileTrue(suck);
    Constants.OperatorConstants.button4.whileTrue(amp);
    Constants.OperatorConstants.pancakeUp.whileTrue(new RaiseLeftElevator());
    Constants.OperatorConstants.pancakeDown.whileTrue(new LeftElevator());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
    // return Autos.exampleAuto();
  }
}
