// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.BlueAlliance;


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
public class Blue3AmpHailMary extends SequentialCommandGroup {       
  SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

  
      // contruct command to follow trajectory
      SwerveControllerCommand blueOrginToAmp = new SwerveControllerCommand(
        AmpTrajectories.tragBlueOriginToAmp, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        AutoConstants.xController, 
        AutoConstants.yController,
        AutoConstants.thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);


      SwerveControllerCommand blueAmpToAmpN = new SwerveControllerCommand(
        AmpTrajectories.tragBlueAmpToAmpNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        AutoConstants.xController, 
        AutoConstants.yController,
        AutoConstants.thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand blueAmpNToAmp = new SwerveControllerCommand(
        AmpTrajectories.tragBlueAmpNoteToAmp, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        AutoConstants.xController, 
        AutoConstants.yController,
        AutoConstants.thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);
      


      SwerveControllerCommand blueAmpToRightmostNote = new SwerveControllerCommand(
        AmpTrajectories.tragBlueAmpToHailMaryNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        AutoConstants.xController, 
        AutoConstants.yController,
        AutoConstants.thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand blueRightmostNoteToAmpM1 = new SwerveControllerCommand(
        AmpTrajectories.tragBlueHailMaryNoteToAmpM1, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        AutoConstants.xController, 
        AutoConstants.yController,
        AutoConstants.thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);


  public Blue3AmpHailMary() {
    addCommands(
        // Reset odometry to starting pose. 
        new InstantCommand(() -> swerveSubsystem.resetOdometry(AmpTrajectories.tragBlueOriginToAmp.getInitialPose())),
        blueOrginToAmp,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(AmpTrajectories.tragBlueAmpToAmpNote.getInitialPose())),
        blueAmpToAmpN,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(AmpTrajectories.tragBlueAmpNoteToAmp.getInitialPose())),
        blueAmpNToAmp,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(AmpTrajectories.tragBlueAmpToHailMaryNote.getInitialPose())),
        blueAmpToRightmostNote,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(AmpTrajectories.tragBlueHailMaryNoteToAmpM1.getInitialPose())),
        new WaitCommand(0.25),
        blueRightmostNoteToAmpM1,
        new InstantCommand(() -> swerveSubsystem.stopModules())

      );
  }
}
