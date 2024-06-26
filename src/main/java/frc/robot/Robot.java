// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private Timer octopusTimer;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // CameraServer.startAutomaticCapture(0);
    m_robotContainer = new RobotContainer();
    SmartDashboard.putString("Messages lol", Constants.SandwichConstants.noteMessage);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for ifunctionatliyems like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Constants.SandwichConstants.noteMessage = "Let's go!!!!!";
    octopusTimer = new Timer();
    octopusTimer.reset();
    octopusTimer.start();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    SmartDashboard.putString("Messages lol", Constants.SandwichConstants.noteMessage);
    SmartDashboard.putString("Octopus Garden ᕕ(⌐■_■)ᕗ ♪♬", Constants.SandwichConstants.octopusLyric);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if(octopusTimer.get() > 1.5 && octopusTimer.get() < 3) {
      Constants.SandwichConstants.octopusLyric = "under the sea...";
    }
    if(octopusTimer.get() > 3 && octopusTimer.get() < 4.5) {
      Constants.SandwichConstants.octopusLyric = "In an octopus's garden...";
    }
    if(octopusTimer.get() > 4.5 && octopusTimer.get() < 6) {
      Constants.SandwichConstants.octopusLyric = "in the shade. doo doo doo";
    }
    if(octopusTimer.get() > 6 && octopusTimer.get() < 7.5) {
      Constants.SandwichConstants.octopusLyric = "He'd let us in,";
    }
    if(octopusTimer.get() > 7.5 && octopusTimer.get() < 9) {
      Constants.SandwichConstants.octopusLyric = "knows where we've been";
    }
    if(octopusTimer.get() > 9 && octopusTimer.get() < 10.5) {
      Constants.SandwichConstants.octopusLyric = "in an octopus's garden";
    }
    if(octopusTimer.get() > 10.5 && octopusTimer.get() < 13) {
      Constants.SandwichConstants.octopusLyric = "in the shade";
    }
    if(octopusTimer.get() > 13 && octopusTimer.get() < 15) {
      Constants.SandwichConstants.octopusLyric = "Please auto work!";
    }
    SmartDashboard.putString("Octopus Garden", Constants.SandwichConstants.octopusLyric);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putString("Messages lol", Constants.SandwichConstants.noteMessage);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
