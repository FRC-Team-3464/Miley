// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.Random;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PivoterConstants;
import frc.robot.Constants.SandwichConstants;
import frc.robot.commands.Pivoter.PIDPivotToPosition;
import frc.robot.commands.Pivoter.PivotToPosition;
import frc.robot.commands.ShooterIntake.RunIntake;
import frc.robot.commands.ShooterIntake.ShooterVelocityPID;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RussianRoulette extends SequentialCommandGroup {
  /** Creates a new RussianRoulette. */
  
  public RussianRoulette() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new AutoPositionToby(2),
        new PIDPivotToPosition(PivoterConstants.kAmpStagePivoterRotations),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new ShooterVelocityPID(SandwichConstants.kShootVelocityTarget)
        )
      ),
      new RunIntake(SandwichConstants.kTriggerIntakeSpeed)
    );
  }
}
