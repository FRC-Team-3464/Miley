// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.RedAlliance;


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
import frc.robot.commands.ShooterIntake.ReverseIntake;
import frc.robot.commands.ShooterIntake.ShootManual;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.trajectories.SpeakerTrajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Red2StraightSpeaker extends SequentialCommandGroup {       

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

  SwerveControllerCommand speakerNoteToOrigin = new SwerveControllerCommand(
    SpeakerTrajectories.tragSpeakerNoteToOrigin, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);

 

 
  public Red2StraightSpeaker() {
    addCommands(
      // GO STRAIGHT FORWARD TO STAGE NOTE AFTER SHOOTING
      new InstantCommand(() -> swerveSubsystem.stopModules()),
      new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragSpeakerShootingToSpeakerNote.getInitialPose())),  
      new PIDPivotToPosition(PivoterConstants.kSubwofferPivoterRotations),
      new ParallelRaceGroup(
        new ShootManual(),
        new WaitCommand(2)        
      ),

      // Go to intake pos
      new PIDPivotToPosition(0),

      // Go to next note pos while intaking. 
      new ParallelCommandGroup(
        new ParallelRaceGroup(
          // Ends when intake done or 3 seconds. 
          new IntakeFromGround(),
          new WaitCommand(3)
        ),
        speakerShootingToSpeakerNote
      ),

      new InstantCommand(() -> swerveSubsystem.stopModules()),
      new WaitCommand(0.25),

      // Go to subwofferr
      // new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragSpeakerNoteToOrigin.getInitialPose())),
      // new ParallelCommandGroup(
      //   new SequentialCommandGroup(
      //     speakerNoteToOrigin, 
      //     new InstantCommand(() -> swerveSubsystem.stopModules()),
      
      // new InstantCommand(() -> swerveSubsystem.stopModules()))),

      // Aim and shoot
      new PIDPivotToPosition(PivoterConstants.kStagePivoterRotations),
      new ParallelRaceGroup(
        // Stop our intake after point 0.1 second - so we're not too high
        new WaitCommand(0.1), 
        new ReverseIntake()
        ),

      // ADD A Wait Command
      new WaitCommand(0.1),

       new ParallelRaceGroup(
        new ShootManual(),
        new WaitCommand(2)        
      ),

      new PIDPivotToPosition(0)
    );
      // new InstantCommand(() -> swerveSubsystem.resetOdometry(SpeakerTrajectories.tragSpeakerNoteToOrigin.getInitialPose())),
      // speakerNoteToOrigin,
      // new InstantCommand(() -> swerveSubsystem.stopModules()),

      // // Aim and shoot
      // // new PIDPivotToPosition(PivoterConstants.kSubwofferPivoterRotations),
      // new PIDPivotToPosition(PivoterConstants.kStagePivoterRotations),
      // new ParallelRaceGroup(
      //   new ShootManual(),
      //   new WaitCommand(2)        
      // ),

      // new PIDPivotToPosition(0)
    
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
  }
}
