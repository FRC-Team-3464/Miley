// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.RedAlliance;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.trajectories.SpeakerTrajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Red3Speaker extends SequentialCommandGroup {       

  SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  
  SwerveControllerCommand originToStageNote = new SwerveControllerCommand(
    SpeakerTrajectories.tragOriginToStageNote, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);
    
SwerveControllerCommand stageNoteToSpeakerShooting = new SwerveControllerCommand(
    SpeakerTrajectories.tragStageNoteToSpeakerShooting, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);

  SwerveControllerCommand speakerShootingToSpeakerNote = new SwerveControllerCommand(
    SpeakerTrajectories.tragSpeakerShootingToSpeakerNote, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);

  SwerveControllerCommand speakerNoteToAmpShooting = new SwerveControllerCommand(
    SpeakerTrajectories.tragSpeakerNoteToAmpShooting, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);
  
  SwerveControllerCommand ampShootingToAmpNote = new SwerveControllerCommand(
    SpeakerTrajectories.tragAmpShootingToAmpNote, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);

  SwerveControllerCommand ampNoteRotateToSpeaker = new SwerveControllerCommand(
    SpeakerTrajectories.tragAmpNoteRotateToSpeaker, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);


 
  /*
   * AMP Trajectories
   */

  public Red3Speaker() {
    addCommands(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragOriginToStageNote.getInitialPose())),
        originToStageNote,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        // Go to Speaker Note
        new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragStageNoteToSpeakerShooting.getInitialPose())),
        stageNoteToSpeakerShooting,
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragSpeakerShootingToSpeakerNote.getInitialPose())),
        speakerShootingToSpeakerNote,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),

        // Go to Amp Note
        new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragSpeakerNoteToAmpShooting.getInitialPose())),
        speakerNoteToAmpShooting,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragAmpShootingToAmpNote.getInitialPose())),
        ampShootingToAmpNote,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        // Rotate to speaker
        new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragAmpNoteRotateToSpeaker.getInitialPose())),
        ampNoteRotateToSpeaker,
        new InstantCommand(() -> swerveSubsystem.stopModules())
      );
  }
}