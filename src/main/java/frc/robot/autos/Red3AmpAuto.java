// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.trajectories.AmpTrajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Red3AmpAuto extends SequentialCommandGroup {
  /** Creates a new Red3AmpAuto. */
 
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);

    // Profiled PID Controller = PID Controller with constraints on max speed / acceleration. 
    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController,
      0,
      0,
      AutoConstants.kThetaControllerConstraints);



  //   // contruct command to follow trajectory
  // SwerveControllerCommand orginToAmp = new SwerveControllerCommand(
  //   AmpTrajectories.tragOriginToAmp, 
  //   swerveSubsystem::getPose, // Coords
  //   DriveConstants.kDriveKinematics, 
  //   xController, 
  //   yController,
  //   thetaController,
  //   swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
  //   swerveSubsystem);

  // SwerveControllerCommand ampToSpeaker = new SwerveControllerCommand(
  //   AmpTrajectories.tragAmpToSpeakerNote, 
  //   swerveSubsystem::getPose, // Coords
  //   DriveConstants.kDriveKinematics, 
  //   xController, 
  //   yController,
  //   thetaController,
  //   swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
  //   swerveSubsystem);


  // SwerveControllerCommand ampToAmpN = new SwerveControllerCommand(
  //   AmpTrajectories.tragAmpToAmpNote, 
  //   swerveSubsystem::getPose, // Coords
  //   DriveConstants.kDriveKinematics, 
  //   xController, 
  //   yController,
  //   thetaController,
  //   swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
  //   swerveSubsystem);

  // SwerveControllerCommand ampNToAmp = new SwerveControllerCommand(
  //   AmpTrajectories.tragAmpNoteToAmp, 
  //   swerveSubsystem::getPose, // Coords
  //   DriveConstants.kDriveKinematics, 
  //   xController, 
  //   yController,
  //   thetaController,
  //   swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
  //   swerveSubsystem);
  
  // SwerveControllerCommand speakerNoteToAmp = new SwerveControllerCommand(
  //   AmpTrajectories.tragSpeakerNoteToAmp, 
  //   swerveSubsystem::getPose, // Coords
  //   DriveConstants.kDriveKinematics, 
  //   xController, 
  //   yController,
  //   thetaController,
  //   swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
  //   swerveSubsystem);

  // SwerveControllerCommand ampToRightmostNote = new SwerveControllerCommand(
  //   AmpTrajectories.tragAmpToHailMaryNote, 
  //   swerveSubsystem::getPose, // Coords
  //   DriveConstants.kDriveKinematics, 
  //   xController, 
  //   yController,
  //   thetaController,
  //   swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
  //   swerveSubsystem);

  // SwerveControllerCommand rightmostNoteToAmpM1 = new SwerveControllerCommand(
  //   AmpTrajectories.tragHailMaryNoteToAmpM1, 
  //   swerveSubsystem::getPose, // Coords
  //   DriveConstants.kDriveKinematics, 
  //   xController, 
  //   yController,
  //   thetaController,
  //   swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
  //   swerveSubsystem);





  public Red3AmpAuto() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);  
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
