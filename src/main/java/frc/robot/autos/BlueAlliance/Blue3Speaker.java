// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.BlueAlliance;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PivoterConstants;
import frc.robot.commands.Pivoter.PIDPivotToPosition;
import frc.robot.commands.ShooterIntake.IntakeFromGround;
import frc.robot.commands.ShooterIntake.ShootManual;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.trajectories.SpeakerTrajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Blue3Speaker extends SequentialCommandGroup {       

  SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  
  SwerveControllerCommand blueOriginToStageNote = new SwerveControllerCommand(
    SpeakerTrajectories.tragBlueOriginToStageNote, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);
    
SwerveControllerCommand blueStageNoteToSpeakerShooting = new SwerveControllerCommand(
    SpeakerTrajectories.tragBlueStageNoteToSpeakerShooting, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);

  SwerveControllerCommand blueSpeakerShootingToSpeakerNote = new SwerveControllerCommand(
    SpeakerTrajectories.tragBlueSpeakerShootingToSpeakerNote, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);

  SwerveControllerCommand blueSpeakerNoteToAmpShooting = new SwerveControllerCommand(
    SpeakerTrajectories.tragBlueSpeakerNoteToAmpShooting, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);
  
  SwerveControllerCommand blueAmpShootingToAmpNote = new SwerveControllerCommand(
    SpeakerTrajectories.tragBlueAmpShootingToAmpNote, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);

  SwerveControllerCommand blueAmpNoteRotateToSpeaker = new SwerveControllerCommand(
    SpeakerTrajectories.tragBlueAmpNoteRotateToSpeaker, 
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

  public Blue3Speaker() {
    addCommands(
          new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragOriginToStageNote.getInitialPose())),  
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new PIDPivotToPosition(PivoterConstants.kSubwofferPivoterRotations),
          new ParallelRaceGroup(
            new ShootManual(),
            new WaitCommand(2)        
          ),
          // Go to subwoffer pos;
          new PIDPivotToPosition(0),

          // Go to next note pos while intaking. 
          new ParallelCommandGroup(
            new ParallelRaceGroup(
              // Ends when intake done or 2 seconds. 
              new IntakeFromGround(),
              new WaitCommand(2)
            ),
            blueOriginToStageNote
          ),

          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),

          // Go to subwoffer
          new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragStageNoteToSpeakerShooting.getInitialPose())),
          blueStageNoteToSpeakerShooting,
          new InstantCommand(() -> swerveSubsystem.stopModules()),

          // Aim and shoot
          new PIDPivotToPosition(PivoterConstants.kSubwofferPivoterRotations),
          new ParallelRaceGroup(
            new ShootManual(),
            new WaitCommand(2)        
          ),
          new PIDPivotToPosition(0),
          new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragSpeakerShootingToSpeakerNote.getInitialPose())),

          // Go to next note pos while intaking. 
          new ParallelCommandGroup(
            new ParallelRaceGroup(
              // Ends when intake done or 5 seconds. 
              new IntakeFromGround(),
              new WaitCommand(2)
            ),
            blueSpeakerShootingToSpeakerNote
          ),
          new InstantCommand(() -> swerveSubsystem.stopModules())
          );
      /*
       * 
       * UNFINISHED
       */
      //   new WaitCommand(0.25),
      //   new InstantCommand(() -> swerveSubsystem.stopModules()),
      //   new WaitCommand(0.25),
      //   new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragSpeakerShootingToSpeakerNote.getInitialPose())),
      //   speakerShootingToSpeakerNote,
      //   new InstantCommand(() -> swerveSubsystem.stopModules()),
      //   new WaitCommand(0.25),

      //   // Go to Amp Note
      //   new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragSpeakerNoteToAmpShooting.getInitialPose())),
      //   speakerNoteToAmpShooting,
      //   new InstantCommand(() -> swerveSubsystem.stopModules()),
      //   new WaitCommand(0.25),
      //   new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragAmpShootingToAmpNote.getInitialPose())),
      //   ampShootingToAmpNote,
      //   new InstantCommand(() -> swerveSubsystem.stopModules()),
      //   new WaitCommand(0.25),
      //   // Rotate to speaker
      //   new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragAmpNoteRotateToSpeaker.getInitialPose())),
      //   ampNoteRotateToSpeaker,
      //   new InstantCommand(() -> swerveSubsystem.stopModules())
      // );

      /*
       * 
       * ORIGINAL
       * 
       */
      // addCommands(
      //       // Reset odometry to starting pose. 
      //       new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragBlueOriginToStageNote.getInitialPose())),
      //       blueOriginToStageNote,
      //       new InstantCommand(() -> swerveSubsystem.stopModules()),
      //       new WaitCommand(0.25),
      //       // Go to Speaker Note
      //       new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragBlueStageNoteToSpeakerShooting.getInitialPose())),
      //       blueStageNoteToSpeakerShooting,
      //       new WaitCommand(0.25),
      //       new InstantCommand(() -> swerveSubsystem.stopModules()),
      //       new WaitCommand(0.25),
      //       new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragBlueSpeakerShootingToSpeakerNote.getInitialPose())),
      //       blueSpeakerShootingToSpeakerNote,
      //       new InstantCommand(() -> swerveSubsystem.stopModules()),
      //       new WaitCommand(0.25),

      //       // Go to Amp Note
      //       new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragBlueSpeakerNoteToAmpShooting.getInitialPose())),
      //       blueSpeakerNoteToAmpShooting,
      //       new InstantCommand(() -> swerveSubsystem.stopModules()),
      //       new WaitCommand(0.25),
      //       new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragBlueAmpShootingToAmpNote.getInitialPose())),
      //       blueAmpShootingToAmpNote,
      //       new InstantCommand(() -> swerveSubsystem.stopModules()),
      //       new WaitCommand(0.25),
      //       // Rotate to speaker
      //       new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragBlueAmpNoteRotateToSpeaker.getInitialPose())),
      //       blueAmpNoteRotateToSpeaker,
      //       new InstantCommand(() -> swerveSubsystem.stopModules())
      //       // new WaitCommand(0.25)

      //   );
  }
}
