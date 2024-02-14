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
import frc.robot.trajectories.AmpTrajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Red3AmpHailMary extends SequentialCommandGroup {       
  SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  SwerveControllerCommand orginToAmp = new SwerveControllerCommand(
    AmpTrajectories.tragOriginToAmp, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);

  SwerveControllerCommand ampToAmpN = new SwerveControllerCommand(
    AmpTrajectories.tragAmpToAmpNote, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);

  SwerveControllerCommand ampNToAmp = new SwerveControllerCommand(
    AmpTrajectories.tragAmpNoteToAmp, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);
  

  SwerveControllerCommand ampToRightmostNote = new SwerveControllerCommand(
    AmpTrajectories.tragAmpToHailMaryNote, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);

  SwerveControllerCommand rightmostNoteToAmpM1 = new SwerveControllerCommand(
    AmpTrajectories.tragHailMaryNoteToAmpM1, 
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

  public Red3AmpHailMary() {
    addCommands(
        // Reset odometry to starting pose. 
        new InstantCommand(() -> swerveSubsystem.resetOdometry(AmpTrajectories.tragOriginToAmp.getInitialPose())),
        orginToAmp,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(AmpTrajectories.tragAmpToAmpNote.getInitialPose())),
        ampToAmpN,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(AmpTrajectories.tragAmpNoteToAmp.getInitialPose())),
        ampNToAmp,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(AmpTrajectories.tragAmpToHailMaryNote.getInitialPose())),
        ampToRightmostNote,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(AmpTrajectories.tragHailMaryNoteToAmpM1.getInitialPose())),
        new WaitCommand(0.25),
        rightmostNoteToAmpM1,
        new InstantCommand(() -> swerveSubsystem.stopModules())
      );
  }
}
