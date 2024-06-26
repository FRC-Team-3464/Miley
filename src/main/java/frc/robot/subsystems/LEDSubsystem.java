// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
  private int firstRedV = 1;
  private static LEDSubsystem instance = null;  
  private int rainbowFirstPixelHue = 1;
  String ledState;

  public LEDSubsystem() {
    // PWM port 0
    // Must be a PWM header, not MXP or DIO
    ledStrip = new AddressableLED(0);
    ledState = "starting";


    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    ledBuffer = new AddressableLEDBuffer(14);
    ledStrip.setLength(ledBuffer.getLength());


    // Set the data
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  public static LEDSubsystem getInstance() {
    if (instance == null) {
      instance = new LEDSubsystem();
    }  
    return instance;
  }


  public String getLEDState(){
    return ledState;
  }

  public void setOff(){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0,0,0);
    }
   ledState = "off";
   ledStrip.setData(ledBuffer); 
  }

  public void redPulse() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {  
      final var huet = (firstRedV + (i * 180 / ledBuffer.getLength())) % 250;
      ledBuffer.setHSV(i, 1, 255, huet);
    }
    firstRedV = (firstRedV + 5) % 250;
    ledStrip.setData(ledBuffer);
  }

  public void bluePulse() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {  
      final var huet = (firstRedV + (i * 180 / (Math.floorDiv(ledBuffer.getLength(), 3)))) % 250;
      ledBuffer.setHSV(i, 100, 255, huet);
    }
    firstRedV = (firstRedV + 10) % 250;
    ledStrip.setData(ledBuffer);
  }

  public void bluePulseReverse() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {  
      final var huet = (firstRedV + (i * 180 / (Math.floorDiv(ledBuffer.getLength(), 3)))) % 250;
      ledBuffer.setHSV(i, 100, 255, huet);
    }
    firstRedV = (firstRedV - 10) % 250;
    ledStrip.setData(ledBuffer);
  }

  public void rainbow() {
    // For every pixel
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Set the value
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    // // Increase by to make the rainbow "move"
    rainbowFirstPixelHue = (rainbowFirstPixelHue + 5) % 180;
    // // Check bounds
    // rainbowFirstPixelHue %= 180;
    ledStrip.setData(ledBuffer);
  }

  public void setWhite() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 74, 65, 42);
     }
      ledState = "white";
     ledStrip.setData(ledBuffer);
  }
  
  public void setRed() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i,240,50,50);
    }
    ledState = "Red";
    ledStrip.setData(ledBuffer);
  }

    public void setGreen() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 0, 65, 0);
     }
      ledState = "Green";
     ledStrip.setData(ledBuffer);
  }


  public void setOrange() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 255, 65, 0);
     }
      ledState = "Orange";
     ledStrip.setData(ledBuffer);
  }

    public void setYellow() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 255, 255, 0);
     }
      ledState = "Yellow";
     ledStrip.setData(ledBuffer);
  }


  public void setBlue() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i,50,50,240);
    }
    ledState = "Blue";
    ledStrip.setData(ledBuffer);
  }



  public void setPurple() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i,150,0,150);
    }
    ledState = "Purple";
    ledStrip.setData(ledBuffer);
  }







  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // setOrange();
    SmartDashboard.putString("LedState", ledState);
 

  }
}
