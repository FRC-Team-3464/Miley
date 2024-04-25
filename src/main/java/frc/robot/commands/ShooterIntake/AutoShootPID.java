// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Constants;
import frc.robot.Constants.SandwichConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// PID Command to shoot note. 
public class AutoShootPID extends SequentialCommandGroup {
  public AutoShootPID() {
    addCommands(
      new ShooterVelocityPID(SandwichConstants.kShootVelocityTarget), 
      new ParallelRaceGroup(
        new ShootPIDEnd(), 
        new RunIntake(SandwichConstants.kTriggerIntakeSpeed)));
  }
}
