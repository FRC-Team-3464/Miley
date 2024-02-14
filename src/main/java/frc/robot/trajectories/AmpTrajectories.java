package frc.robot.trajectories;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TragConstants;

public class AmpTrajectories {
    
  /*
   * AMP Trajectories
   */

    public static final Trajectory tragOriginToAmp = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          TragConstants.coordsOriginToAmpStart[0],
          TragConstants.coordsOriginToAmpStart[1],
          Rotation2d.fromDegrees(TragConstants.coordsOriginToAmpStart[2])), 
        new Pose2d(
          TragConstants.coordsOriginToAmpEnd[0],
          TragConstants.coordsOriginToAmpEnd[1],
          Rotation2d.fromDegrees(TragConstants.coordsOriginToAmpEnd[2]))
      ),
      AutoConstants.trajectoryConfig); // Apply trajectory settings to path

    public static final Trajectory tragAmpToAmpNote = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          TragConstants.coordsAmpToAmpNoteStart[0],
          TragConstants.coordsAmpToAmpNoteStart[1], 
          Rotation2d.fromDegrees(TragConstants.coordsAmpToAmpNoteStart[2])), 
        new Pose2d(
          TragConstants.coordsAmpToAmpNoteWayPoint1[0], 
          TragConstants.coordsAmpToAmpNoteWayPoint1[1], 
          Rotation2d.fromDegrees(TragConstants.coordsAmpToAmpNoteWayPoint1[2])),       
        new Pose2d(
          TragConstants.coordsAmpToAmpNoteEnd[0], 
          TragConstants.coordsAmpToAmpNoteEnd[1],
          Rotation2d.fromDegrees(TragConstants.coordsAmpToAmpNoteEnd[2]))
      ), 
      AutoConstants.trajectoryConfig);
      
    //  Trag Amp Note to Amp is simply each point of AmpToAmpNote Trag, except the x and y values are flipped. 
    public static final Trajectory tragAmpNoteToAmp = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          // Should be (0, 0, [Heading of Previous])%
          -TragConstants.coordsAmpToAmpNoteStart[0],
          -TragConstants.coordsAmpToAmpNoteStart[1], 
          // Heading is same from the end of the previous command
          Rotation2d.fromDegrees(TragConstants.coordsAmpToAmpNoteEnd[2])), 
        new Pose2d(
          -TragConstants.coordsAmpToAmpNoteWayPoint1[0], 
          -TragConstants.coordsAmpToAmpNoteWayPoint1[1], 
          // Heading is same from the start of the previous
          Rotation2d.fromDegrees(TragConstants.coordsOriginToAmpStart[2])),       
        new Pose2d(
          -TragConstants.coordsAmpToAmpNoteEnd[0], 
          -TragConstants.coordsAmpToAmpNoteEnd[1],
          Rotation2d.fromDegrees(TragConstants.coordsOriginToAmpStart[2]))
      ), 
      AutoConstants.trajectoryConfig);

    public static final Trajectory tragAmpToSpeakerNote = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          TragConstants.coordsAmpToSpeakerNoteStart[0],
          TragConstants.coordsAmpToSpeakerNoteStart[1], 
          Rotation2d.fromDegrees(TragConstants.coordsAmpToSpeakerNoteStart[2])), 
        new Pose2d(
          TragConstants.coordsAmpToSpeakerNoteWayPoint1[0], 
          TragConstants.coordsAmpToSpeakerNoteWayPoint1[1], 
          Rotation2d.fromDegrees(TragConstants.coordsAmpToSpeakerNoteWayPoint1[2])),       
        new Pose2d(
          TragConstants.coordsAmpToSpeakerNoteEnd[0], 
          TragConstants.coordsAmpToSpeakerNoteEnd[1],
          Rotation2d.fromDegrees(TragConstants.coordsAmpToSpeakerNoteEnd[2]))  
      ),
      AutoConstants.trajectoryConfig);

    public static final Trajectory tragSpeakerNoteToAmp = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          // Should be (0,0,0)
          TragConstants.coordsAmpToSpeakerNoteStart[0],
          TragConstants.coordsAmpToSpeakerNoteStart[1], 
          Rotation2d.fromDegrees(TragConstants.coordsAmpToSpeakerNoteEnd[2])), 
        new Pose2d(
          // Ignore coping the Waypoint [Amp to Source Note]
          // This trajectory allows us to hit the amp dead on. 
          -TragConstants.coordsAmpToSpeakerNoteEnd[0],
          -TragConstants.coordsAmpToSpeakerNoteEnd[1] / 2, 
          Rotation2d.fromDegrees(TragConstants.coordsAmpToSpeakerNoteWayPoint1[2])),       
        new Pose2d(
          -TragConstants.coordsAmpToSpeakerNoteEnd[0], 
          -TragConstants.coordsAmpToSpeakerNoteEnd[1],
          Rotation2d.fromDegrees(TragConstants.coordsAmpToSpeakerNoteStart[2]))  
      ),
      AutoConstants.trajectoryConfig);

    public static final Trajectory tragAmpToHailMaryNote = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          // Should be (0,0,0)
          TragConstants.coordsAmpToHailMaryNoteStart[0],
          TragConstants.coordsAmpToHailMaryNoteStart[1], 
          Rotation2d.fromDegrees(TragConstants.coordsAmpToHailMaryNoteStart[2])), 
        new Pose2d(
          TragConstants.coordsAmpToHailMaryNoteWayPoint1[0],
          TragConstants.coordsAmpToHailMaryNoteWayPoint1[1], 
          Rotation2d.fromDegrees(TragConstants.coordsAmpToHailMaryNoteWayPoint1[2])),       
        new Pose2d(
          TragConstants.coordsAmpToHailMaryNoteEnd[0], 
          TragConstants.coordsAmpToHailMaryNoteEnd[1],
          Rotation2d.fromDegrees(TragConstants.coordsAmpToHailMaryNoteEnd[2]))  
      ),
      AutoConstants.trajectoryConfig);

    public static final Trajectory tragHailMaryNoteToAmpM1 = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          // Should be (0,0,0)
          TragConstants.coordsHailMaryNoteToAmpStart[0],
          TragConstants.coordsHailMaryNoteToAmpStart[1], 
          Rotation2d.fromDegrees(TragConstants.coordsHailMaryNoteToAmpStart[2])), 
        new Pose2d(
          // Not sure about rotation
          TragConstants.coordsHailMaryNoteToAmpWayPoint1[0],
          TragConstants.coordsHailMaryNoteToAmpWayPoint1[1], 
          Rotation2d.fromDegrees(TragConstants.coordsHailMaryNoteToAmpWayPoint1[2])),       
        new Pose2d(
          TragConstants.coordsHailMaryNoteToAmpEnd[0], 
          TragConstants.coordsHailMaryNoteToAmpEnd[1],
          Rotation2d.fromDegrees(TragConstants.coordsHailMaryNoteToAmpEnd[2]))  
      ),
      AutoConstants.trajectoryConfig);

    /*
     * 
     * Blue Alliance AMP Trajectories
     * 
     */
    
    public static final Trajectory tragBlueOriginToAmp = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          TragConstants.coordsOriginToAmpStart[0],
          -TragConstants.coordsOriginToAmpStart[1],
          Rotation2d.fromDegrees(-TragConstants.coordsOriginToAmpStart[2])), 
        new Pose2d(
          TragConstants.coordsOriginToAmpEnd[0],
          -TragConstants.coordsOriginToAmpEnd[1],
          Rotation2d.fromDegrees(-TragConstants.coordsOriginToAmpEnd[2]))
      ),
      AutoConstants.trajectoryConfig); // Apply trajectory settings to path

    
    public static final Trajectory tragBlueAmpToAmpNote = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          TragConstants.coordsAmpToAmpNoteStart[0],
          -TragConstants.coordsAmpToAmpNoteStart[1], 
          Rotation2d.fromDegrees(-TragConstants.coordsAmpToAmpNoteStart[2])), 
        new Pose2d(
          TragConstants.coordsAmpToAmpNoteWayPoint1[0], 
          -TragConstants.coordsAmpToAmpNoteWayPoint1[1], 
          Rotation2d.fromDegrees(-TragConstants.coordsAmpToAmpNoteWayPoint1[2])),       
        new Pose2d(
          TragConstants.coordsAmpToAmpNoteEnd[0], 
          -TragConstants.coordsAmpToAmpNoteEnd[1],
          Rotation2d.fromDegrees(-TragConstants.coordsAmpToAmpNoteEnd[2]))
      ), 
      AutoConstants.trajectoryConfig);


    //  Trag Amp Note to Amp is simply each point of AmpToAmpNote Trag, except the x and y values are flipped. 
    public static final Trajectory tragBlueAmpNoteToAmp = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          // Should be (0, 0, [Heading of Previous])%
          -TragConstants.coordsAmpToAmpNoteStart[0],
          TragConstants.coordsAmpToAmpNoteStart[1], 
          // Heading is same from the end of the previous command
          Rotation2d.fromDegrees(-TragConstants.coordsAmpToAmpNoteEnd[2])), 
        new Pose2d(
          -TragConstants.coordsAmpToAmpNoteWayPoint1[0], 
          TragConstants.coordsAmpToAmpNoteWayPoint1[1], 
          // Heading is same from the start of the previous
          Rotation2d.fromDegrees(-TragConstants.coordsOriginToAmpStart[2])),       
        new Pose2d(
          -TragConstants.coordsAmpToAmpNoteEnd[0], 
          TragConstants.coordsAmpToAmpNoteEnd[1],
          Rotation2d.fromDegrees(-TragConstants.coordsOriginToAmpStart[2]))
      ), 
      AutoConstants.trajectoryConfig);


    public static final Trajectory tragBlueAmpToSpeakerNote = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          TragConstants.coordsAmpToSpeakerNoteStart[0],
          -TragConstants.coordsAmpToSpeakerNoteStart[1], 
          Rotation2d.fromDegrees(-TragConstants.coordsAmpToSpeakerNoteStart[2])), 
        new Pose2d(
          TragConstants.coordsAmpToSpeakerNoteWayPoint1[0], 
          -TragConstants.coordsAmpToSpeakerNoteWayPoint1[1], 
          Rotation2d.fromDegrees(-TragConstants.coordsAmpToSpeakerNoteWayPoint1[2])),       
        new Pose2d(
          TragConstants.coordsAmpToSpeakerNoteEnd[0], 
          -TragConstants.coordsAmpToSpeakerNoteEnd[1],
          Rotation2d.fromDegrees(-TragConstants.coordsAmpToSpeakerNoteEnd[2]))  
      ),
      AutoConstants.trajectoryConfig);


    public static final Trajectory tragBlueSpeakerNoteToAmp = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          // Should be (0,0,0)
          TragConstants.coordsAmpToSpeakerNoteStart[0],
          -TragConstants.coordsAmpToSpeakerNoteStart[1], 
          Rotation2d.fromDegrees(-TragConstants.coordsAmpToSpeakerNoteEnd[2])), 
        new Pose2d(
          // Ignore coping the Waypoint [Amp to Source Note]
          // This trajectory allows us to hit the amp dead on. 
          -TragConstants.coordsAmpToSpeakerNoteEnd[0],
          TragConstants.coordsAmpToSpeakerNoteEnd[1] / 2, 
          Rotation2d.fromDegrees(-TragConstants.coordsAmpToSpeakerNoteWayPoint1[2])),       
        new Pose2d(
          -TragConstants.coordsAmpToSpeakerNoteEnd[0], 
          TragConstants.coordsAmpToSpeakerNoteEnd[1],
          Rotation2d.fromDegrees(-TragConstants.coordsAmpToSpeakerNoteStart[2]))  
      ),
      AutoConstants.trajectoryConfig);

    public static final Trajectory tragBlueAmpToHailMaryNote = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          // Should be (0,0,0)
          TragConstants.coordsAmpToHailMaryNoteStart[0],
          -TragConstants.coordsAmpToHailMaryNoteStart[1], 
          Rotation2d.fromDegrees(-TragConstants.coordsAmpToHailMaryNoteStart[2])), 
        new Pose2d(
          TragConstants.coordsAmpToHailMaryNoteWayPoint1[0],
          -TragConstants.coordsAmpToHailMaryNoteWayPoint1[1], 
          Rotation2d.fromDegrees(-TragConstants.coordsAmpToHailMaryNoteWayPoint1[2])),       
        new Pose2d(
          TragConstants.coordsAmpToHailMaryNoteEnd[0], 
          -TragConstants.coordsAmpToHailMaryNoteEnd[1],
          Rotation2d.fromDegrees(-TragConstants.coordsAmpToHailMaryNoteEnd[2]))  
      ),
      AutoConstants.trajectoryConfig);

    public static final Trajectory tragBlueHailMaryNoteToAmpM1 = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(
          // Should be (0,0,0)
          TragConstants.coordsHailMaryNoteToAmpStart[0],
          -TragConstants.coordsHailMaryNoteToAmpStart[1], 
          Rotation2d.fromDegrees(-TragConstants.coordsHailMaryNoteToAmpStart[2])), 
        new Pose2d(
          // Not sure about rotation
          TragConstants.coordsHailMaryNoteToAmpWayPoint1[0],
          -TragConstants.coordsHailMaryNoteToAmpWayPoint1[1], 
          Rotation2d.fromDegrees(-TragConstants.coordsHailMaryNoteToAmpWayPoint1[2])),       
        new Pose2d(
          TragConstants.coordsHailMaryNoteToAmpEnd[0], 
          -TragConstants.coordsHailMaryNoteToAmpEnd[1],
          Rotation2d.fromDegrees(-TragConstants.coordsHailMaryNoteToAmpEnd[2]))  
      ),
      AutoConstants.trajectoryConfig);

}
