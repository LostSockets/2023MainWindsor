package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmTelescopicSubsystem;

public class ArmTelescopicCmd extends CommandBase {

  private final ArmTelescopicSubsystem armTelescopicSubsystem;
  private final double speed;

  public ArmTelescopicCmd(ArmTelescopicSubsystem armTelescopicSubsystem, double speed) {
    this.armTelescopicSubsystem = armTelescopicSubsystem;
    this.speed = speed;
    addRequirements(armTelescopicSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armTelescopicSubsystem.setMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armTelescopicSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
