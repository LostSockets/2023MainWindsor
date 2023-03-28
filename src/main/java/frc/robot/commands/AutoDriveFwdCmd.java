package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveFwdCmd extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final double distance;
  private double encoderSetpoint;

  public AutoDriveFwdCmd(DriveSubsystem driveSubsystem, double distance) {
    this.driveSubsystem = driveSubsystem;
    this.distance = distance;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoderSetpoint = -1 * (driveSubsystem.getEncoderMeters() + distance);
    System.out.print("AutoDriveFwd started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double error = encoderSetpoint - driveSubsystem.getEncoderMeters();
    double outputSpeed = Constants.DriveConstants.kP * error;

    driveSubsystem.setMotors(outputSpeed, outputSpeed);

    //System.out.print("error = ");
    //System.out.println(error);
    //System.out.print("outputSpeed = ");
    //System.out.println(outputSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (driveSubsystem.getEncoderMeters() > Math.abs(encoderSetpoint - 0.1)) {
      System.out.println("AutoDriveFwdCmd complete");
      return true;
    } else {
      return false;
    }
  }
}
