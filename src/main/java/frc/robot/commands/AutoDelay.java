package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

public class AutoDelay extends CommandBase {

private final double timeDelay;
private double startTime = 0;

  public AutoDelay(double timeDelay) {
    this.timeDelay = timeDelay;
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Timer timer = new Timer();
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (Timer.getFPGATimestamp() - startTime > timeDelay){
      // This is just a delay timer!
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
 }
}
