// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSubsystem extends SubsystemBase { 
  private static PhotonSubsystem instance = null;  
  private final PhotonCamera aprilCamera;
  private final PhotonCamera noteCamera;
  ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");

  public PhotonSubsystem() {
    aprilCamera = new PhotonCamera("AprilTagCamera");
    noteCamera = new PhotonCamera("NoteCamera");
  }


  public static PhotonSubsystem getInstance() {
    if (instance == null) {
      instance = new PhotonSubsystem();
    }  
    return instance;
  }

  public PhotonCamera getAprilCamera() {
    return aprilCamera;
  }

  public PhotonCamera getNoteCamera() {
    return noteCamera;
  }

  // public boolean getAprilHasTargets(){
  //   return getAprilCamera().getLatestResult().hasTargets();
  // }

  
  // public boolean getNoteHasTargets(){
  //   return getNoteCamera().getLatestResult().hasTargets();
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // visionTab.addBoolean("Apriltag Target?", this::getAprilHasTargets);
    // SmartDashboard.putBoolean("PhotonHasTarget", aprilCamera.getLatestResult().hasTargets()); 
  }
}
