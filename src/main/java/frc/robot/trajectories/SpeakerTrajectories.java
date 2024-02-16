package frc.robot.trajectories;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TragConstants;

public class SpeakerTrajectories {

    /*
     * 
     * SPEAKER Trajectories
     * 
     */
       
    public static final Trajectory tragOriginToStageNote = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          TragConstants.coordsOriginToStageNoteStart[0], 
          TragConstants.coordsOriginToStageNoteStart[1], 
          Rotation2d.fromDegrees(TragConstants.coordsOriginToStageNoteStart[2])), 
        new Pose2d(
          TragConstants.coordsOriginToStageNoteStartWayPoint1[0], 
          TragConstants.coordsOriginToStageNoteStartWayPoint1[1], 
          Rotation2d.fromDegrees(TragConstants.coordsOriginToStageNoteStartWayPoint1[2])), 
        new Pose2d(
          TragConstants.coordsOriginToStageNoteEnd[0], 
          TragConstants.coordsOriginToStageNoteEnd[1], 
          Rotation2d.fromDegrees(TragConstants.coordsOriginToStageNoteEnd[2]))
      ),
      AutoConstants.trajectoryConfig); 


    public static final Trajectory tragStageNoteToSpeakerShooting = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          TragConstants.coordsStageNoteToSpeakerShootingStart[0], 
          TragConstants.coordsStageNoteToSpeakerShootingStart[1], 
          Rotation2d.fromDegrees(TragConstants.coordsStageNoteToSpeakerShootingStart[2])), 
        new Pose2d(
          TragConstants.coordsStageNoteToSpeakerShootingEnd[0], 
          TragConstants.coordsStageNoteToSpeakerShootingEnd[1], 
          Rotation2d.fromDegrees(TragConstants.coordsStageNoteToSpeakerShootingEnd[2]))
      ),
      AutoConstants.trajectoryConfig); 

    public static final Trajectory tragSpeakerShootingToSpeakerNote = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          TragConstants.coordsSpeakerShootingToSpeakerNoteStart[0], 
          TragConstants.coordsSpeakerShootingToSpeakerNoteStart[1], 
          Rotation2d.fromDegrees(TragConstants.coordsSpeakerShootingToSpeakerNoteStart[2])), 
        new Pose2d(
          TragConstants.coordsSpeakerShootingToSpeakerNoteEnd[0], 
          TragConstants.coordsSpeakerShootingToSpeakerNoteEnd[1], 
          Rotation2d.fromDegrees(TragConstants.coordsSpeakerShootingToSpeakerNoteEnd[2]))
      ),
      AutoConstants.trajectoryConfig); // change X to 1.3464 because team spirit and nationalism 
    

    public static final Trajectory tragSpeakerNoteToAmpShooting = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          TragConstants.coordsSpeakerNoteToAmpShootingStart[0], 
          TragConstants.coordsSpeakerNoteToAmpShootingStart[1], 
          Rotation2d.fromDegrees(TragConstants.coordsSpeakerNoteToAmpShootingStart[2])), 
        new Pose2d(
          TragConstants.coordsSpeakerNoteToAmpShootingEnd[0], 
          TragConstants.coordsSpeakerNoteToAmpShootingEnd[1], 
          Rotation2d.fromDegrees(TragConstants.coordsSpeakerNoteToAmpShootingEnd[2]))
      ),
      AutoConstants.trajectoryConfig); 

    /* This is direct COPY From SpeakerShootingToSpeakerNote, as it's the same translation*/
    public static final Trajectory tragAmpShootingToAmpNote = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          TragConstants.coordsSpeakerShootingToSpeakerNoteStart[0], 
          TragConstants.coordsSpeakerShootingToSpeakerNoteStart[1], 
          Rotation2d.fromDegrees(TragConstants.coordsSpeakerShootingToSpeakerNoteStart[2])), 
        new Pose2d(
          TragConstants.coordsSpeakerShootingToSpeakerNoteEnd[0], 
          TragConstants.coordsSpeakerShootingToSpeakerNoteEnd[1], 
          Rotation2d.fromDegrees(TragConstants.coordsSpeakerShootingToSpeakerNoteEnd[2]))
      ),
      AutoConstants.trajectoryConfig); 
      
    
    public static final Trajectory tragAmpNoteRotateToSpeaker = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          TragConstants.coordsAmpNoteRotateToSpeakerStart[0], 
          TragConstants.coordsAmpNoteRotateToSpeakerStart[1], 
          Rotation2d.fromDegrees(TragConstants.coordsAmpNoteRotateToSpeakerStart[2])), 
        new Pose2d(
          TragConstants.coordsAmpNoteRotateToSpeakerEnd[0],
          TragConstants.coordsAmpNoteRotateToSpeakerEnd[1],
          Rotation2d.fromDegrees(TragConstants.coordsAmpNoteRotateToSpeakerEnd[2]))
      ),
      AutoConstants.trajectoryConfig); // change X to 1.3464 because team spirit and nationalism 

      /*
       * 
       * Blue Alliance Speaker
       * 
       */

      public static final Trajectory tragBlueOriginToStageNote = TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(
            TragConstants.coordsOriginToStageNoteStart[0], 
            -TragConstants.coordsOriginToStageNoteStart[1], 
            Rotation2d.fromDegrees(-TragConstants.coordsOriginToStageNoteStart[2])), 
          new Pose2d(
            TragConstants.coordsOriginToStageNoteStartWayPoint1[0], 
            -TragConstants.coordsOriginToStageNoteStartWayPoint1[1], 
            Rotation2d.fromDegrees(-TragConstants.coordsOriginToStageNoteStartWayPoint1[2])), 
          new Pose2d(
            TragConstants.coordsOriginToStageNoteEnd[0], 
            -TragConstants.coordsOriginToStageNoteEnd[1], 
            Rotation2d.fromDegrees(-TragConstants.coordsOriginToStageNoteEnd[2]))
        ),  
      AutoConstants.trajectoryConfig);  
      
      public static final Trajectory tragBlueStageNoteToSpeakerShooting = TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(
            TragConstants.coordsStageNoteToSpeakerShootingStart[0], 
            -TragConstants.coordsStageNoteToSpeakerShootingStart[1], 
            Rotation2d.fromDegrees(-TragConstants.coordsStageNoteToSpeakerShootingStart[2])), 
          new Pose2d(
            TragConstants.coordsStageNoteToSpeakerShootingEnd[0], 
            -TragConstants.coordsStageNoteToSpeakerShootingEnd[1], 
            Rotation2d.fromDegrees(-TragConstants.coordsStageNoteToSpeakerShootingEnd[2]))
        ),
        AutoConstants.trajectoryConfig); 

      public static final Trajectory tragBlueSpeakerShootingToSpeakerNote = TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(
            TragConstants.coordsSpeakerShootingToSpeakerNoteStart[0], 
            -TragConstants.coordsSpeakerShootingToSpeakerNoteStart[1], 
            Rotation2d.fromDegrees(TragConstants.coordsSpeakerShootingToSpeakerNoteStart[2])), 
          new Pose2d(
            TragConstants.coordsSpeakerShootingToSpeakerNoteEnd[0], 
            -TragConstants.coordsSpeakerShootingToSpeakerNoteEnd[1], 
            Rotation2d.fromDegrees(-TragConstants.coordsSpeakerShootingToSpeakerNoteEnd[2]))
        ),
        AutoConstants.trajectoryConfig); // change X to 1.3464 because team spirit and nationalism 
    


    public static final Trajectory tragBlueSpeakerNoteToAmpShooting = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          TragConstants.coordsSpeakerNoteToAmpShootingStart[0], 
          -TragConstants.coordsSpeakerNoteToAmpShootingStart[1], 
          Rotation2d.fromDegrees(-TragConstants.coordsSpeakerNoteToAmpShootingStart[2])), 
        new Pose2d(
          TragConstants.coordsSpeakerNoteToAmpShootingEnd[0], 
          -TragConstants.coordsSpeakerNoteToAmpShootingEnd[1], 
          Rotation2d.fromDegrees(-TragConstants.coordsSpeakerNoteToAmpShootingEnd[2]))
      ),
      AutoConstants.trajectoryConfig); 

    /* This is direct COPY From SpeakerShootingToSpeakerNote, as it's the same translation*/
    public static final Trajectory tragBlueAmpShootingToAmpNote = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          TragConstants.coordsSpeakerShootingToSpeakerNoteStart[0], 
          -TragConstants.coordsSpeakerShootingToSpeakerNoteStart[1], 
          Rotation2d.fromDegrees(-TragConstants.coordsSpeakerShootingToSpeakerNoteStart[2])), 
        new Pose2d(
          TragConstants.coordsSpeakerShootingToSpeakerNoteEnd[0], 
          -TragConstants.coordsSpeakerShootingToSpeakerNoteEnd[1], 
          Rotation2d.fromDegrees(-TragConstants.coordsSpeakerShootingToSpeakerNoteEnd[2]))
      ),
      AutoConstants.trajectoryConfig); 
      
    
    public static final Trajectory tragBlueAmpNoteRotateToSpeaker = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          TragConstants.coordsAmpNoteRotateToSpeakerStart[0], 
          -TragConstants.coordsAmpNoteRotateToSpeakerStart[1], 
          Rotation2d.fromDegrees(-TragConstants.coordsAmpNoteRotateToSpeakerStart[2])), 
        new Pose2d(
          TragConstants.coordsAmpNoteRotateToSpeakerEnd[0],
          -TragConstants.coordsAmpNoteRotateToSpeakerEnd[1],
          Rotation2d.fromDegrees(-TragConstants.coordsAmpNoteRotateToSpeakerEnd[2]))
      ),
      AutoConstants.trajectoryConfig); // change X to 1.3464 because team spirit and nationalism 


}
