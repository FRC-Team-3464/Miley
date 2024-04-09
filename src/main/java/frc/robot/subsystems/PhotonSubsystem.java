// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSubsystem extends SubsystemBase { 
  private static PhotonSubsystem instance = null;  
  private final PhotonCamera aprilCamera;
  private final PhotonCamera noteCamera;

  public PhotonSubsystem() {
    aprilCamera = new PhotonCamera("Speaker_Camera");
    noteCamera = new PhotonCamera("Note_Camera");
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
