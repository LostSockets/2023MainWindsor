package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmTelescopicSubsystem;

public class AutoTelescopicExtend extends CommandBase {

  private final ArmTelescopicSubsystem armTelescopicSubsystem;
  double lastTimeStamp = 0;
  double startTime = 0;
  double direction;

  public AutoTelescopicExtend(ArmTelescopicSubsystem armTelescopicSubsystem, double direction) {
    this.armTelescopicSubsystem = armTelescopicSubsystem;
    this.direction = direction;
    addRequirements(armTelescopicSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTimeStamp = Timer.getFPGATimestamp();
    startTime = Timer.getFPGATimestamp();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armTelescopicSubsystem.setMotor(Constants.ArmTelescopicConstants.kArmTelescopicSpeedPercentage * direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armTelescopicSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (Timer.getFPGATimestamp() - startTime > Constants.AutoConstants.kTelescopicTime)
      return true;
    else 
      return false;
  }
}
