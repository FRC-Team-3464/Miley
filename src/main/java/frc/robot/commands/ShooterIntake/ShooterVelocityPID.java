package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterVelocityPID extends Command {
  private ShooterSubsystem shootSub;
  private LEDSubsystem ledSub;
  private double setPoint;
  private double error; 

  public ShooterVelocityPID(double target) {
    shootSub = ShooterSubsystem.getInstance();
    ledSub = LEDSubsystem.getInstance();
    addRequirements(shootSub);
    // addRequirements(ledSub);
    setPoint = target;
  }

  @Override
  public void initialize() {
    if(setPoint == 0) {
      ledSub.setOff();
    }
    if(setPoint == Constants.SandwichConstants.kShootVelocityTarget) {
      Constants.SandwichConstants.noteMessage = "Charging up! brrrRrrRrrRrRrRRRRRRR!!!";
    }
  }

  @Override
  public void execute() {
    if(Constants.PivoterConstants.kPivoterTarget == Constants.PivoterConstants.kAmpStagePivoterRotations && setPoint == Constants.SandwichConstants.kShootVelocityTarget) {
      setPoint = 2000;
    }
    shootSub.runShooterPID(setPoint);
    if(error > 500) {
    ledSub.redPulse();
    }
    else {
      ledSub.rainbow();
    }
    error = Math.abs(setPoint - shootSub.getVelocity());
    SmartDashboard.putNumber("Shoot Setpoint", setPoint);
    SmartDashboard.putNumber("Shoot Velocity", shootSub.getVelocity());
    SmartDashboard.putNumber("Shoot Error", error);
  }

  @Override
  public void end(boolean interrupted) {
    ledSub.rainbow();
  }

  @Override
  public boolean isFinished() {
    // End command once we're within tolerance. 
    // return (error < 300);    
    return (error < 300);

  }
}
