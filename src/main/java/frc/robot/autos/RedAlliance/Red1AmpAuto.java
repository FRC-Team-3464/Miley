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
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Pivoter.PIDPivotToPosition;
import frc.robot.commands.ShooterIntake.IntakeFromGround;
import frc.robot.commands.ShooterIntake.ShootAmp;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.trajectories.AmpTrajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Red1AmpAuto extends SequentialCommandGroup {
        
  SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  // contruct command to follow trajectory
  SwerveControllerCommand orginToAmp = new SwerveControllerCommand(
    AmpTrajectories.tragOriginToAmp, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);

  SwerveControllerCommand ampToSpeaker = new SwerveControllerCommand(
    AmpTrajectories.tragAmpToSpeakerNote, 
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
  
  SwerveControllerCommand speakerNoteToAmp = new SwerveControllerCommand(
    AmpTrajectories.tragSpeakerNoteToAmp, 
    swerveSubsystem::getPose, // Coords
    DriveConstants.kDriveKinematics, 
    AutoConstants.xController, 
    AutoConstants.yController,
    AutoConstants.thetaController,
    swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
    swerveSubsystem);

  /** Creates a new Red3AmpAuto. */
 
  /*
   * AMP Trajectories
   */

  public Red1AmpAuto() {
    addCommands(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(AmpTrajectories.tragOriginToAmp.getInitialPose())),
      // Go to AMP while pivoting to AMP Pos.
      new ParallelCommandGroup(
        orginToAmp, 
        new PIDPivotToPosition(Constants.PivoterConstants.kAmpPivoterRotations)
      ),
      new InstantCommand(() -> swerveSubsystem.stopModules()),
      new WaitCommand(0.25),
      
      // Shoot in AMP for 1.5 Seconds 
      new ParallelRaceGroup(
        new WaitCommand(1.5),
        new ShootAmp()),

      // Move arm down to rest pos
      new PIDPivotToPosition(0),
      new InstantCommand(() -> swerveSubsystem.resetOdometry(AmpTrajectories.tragAmpToAmpNote.getInitialPose())),

      // Drive to AMP Note while intaking
      new ParallelCommandGroup(
        new ParallelRaceGroup(
          // Ends when intake done or 3 seconds. 
          new IntakeFromGround(),
          new WaitCommand(3)
        ),
        ampToAmpN
      ),
      new InstantCommand(() -> swerveSubsystem.stopModules())
      );
      
  }
}
