// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  public static final class DriveConstants {

    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 2;
    public static final int kRightMotor1Port = 3;
    public static final int kRightMotor2Port = 4;

    public static final double kEncoderTick2Feet = 0.188679; // = 1/5.3? why? gear ratio?
    //public static final double kEncoderTick2Feet = ((1.0 / 4096.0) * 2 * Math.PI * 0.5);
    //public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.128 * Math.PI;
    public static final double kP = 0.1; 
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double speedThrottle = 0.4;
    public static final double turnThrottle = 0.25;
    public static final double speedTurbo = 1.0;
    public static final double turnTurbo = 0.15;
    public static final double turnMellow = 0.1;

    // NEED TO FIGURE OUT ENCODERS!

  }

  public static final class AutoConstants {

    public static final double kAutoDriveFwdSpeed = 1.5;
    public static final double kAutoDriveFwdDistance = 1.2;
    public static final double kAutoDriveFwdDistance2 = 4.0;

    public static final double kAutoDriveBkwdSpeed = -0.8;
    public static final double kAutoDriveBkwdDistance = -6.0;

    public static final double kTelescopicTime = 2.0;
  }

  public static final class ArmTelescopicConstants {

    public static final int kArmTelescopicMotorPort = 6;

    public static final double kArmTelescopicSpeedPercentage = 0.7;   
  }

  public static final class ArmPivotConstants {

    public static final int kArmPivotMotorPort = 8;

    public static final double kArmPivotSpeedPercentage = 0.7;
    public static final double kArmPivotSpeedPecentageThrottled = 0.15;
 
    public static final double kP = 0.08; //0.08; // PID numbers need to be figured out still!!!!!!!!
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kPButton = 0.2; //0.09;
    public static final double kIButton = 0;
    public static final double kDButton = 0;
  
    //NOTE: Values below are for start position UP. Commented out values are for when start position is DOWN.
    public static final double kmaxArmPivotEncoderHeight = -1.0; //-8.7;
    public static final double kminArmPivotEncoderHeight = 7.5; //-0.1;
    public static final double kArmPivotPos0 = 7.0; //-0.4;
    public static final double kArmPivotPos1 = 5.0; //-3.0;
    public static final double kArmPivotPos2 = 1.0; //-6.0;
    public static final double kArmPivotPos3 = -1.0; //-8.5;

  }

   public static final class OIConstants {

    public static final int kDriverJoystickPort = 0;
    public static final int kArmJoystickPort = 1;

    public static final int kArcadeDriveSpeedAxis = 1;
    public static final int kArcadeDriveTurnAxis = 4;
    public static final int kArcadeDriveReverse = 3;
    public static final int kArcadeDriveTurbo = 6;
    public static final int kArcadeDriveMellow = 5;

    public static final int kArmTelescopicExtend = 6;
    public static final int kArmTelescopicRetract = 5;
    //public static final int kArmPivotManualRaise = 4;
    //public static final int kArmPivotManualLower = 3;
    public static final int kArmPivotPIDPos0Button = 1;
    public static final int kArmPivotPIDPos1Button = 2;
    public static final int kArmPivotPIDPos2Button = 3;
    public static final int kArmPivotPIDPos3Button = 4;
    //public static final int kArmPivotPIDButtonRaise = 6;
    //public static final int kArmPivotPIDButtonLower = 5;
    public static final int kArmPivotAxis = 5;

    public static final int kGripperOpen = 2;
    public static final int kGripperClose = 3;

  }
 
}
