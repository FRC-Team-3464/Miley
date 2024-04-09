// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SandwichConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// PID Command to shoot note. 
public class ShootPID extends SequentialCommandGroup {
  public ShootPID() {
    addCommands(
      // Spit Note Back
      new ParallelRaceGroup(
        new ReverseIntake(),
        new WaitCommand(0.03)

      ),

      // Run the shooter to desired RPM
      new ShooterVelocityPID(SandwichConstants.kShootVelocityTarget));
      // new ParallelRaceGroup(
      //   new RunIntake(1),
      //   new WaitCommand(0.5)),
      // new ShooterVelocityPID(0));
  }
}
