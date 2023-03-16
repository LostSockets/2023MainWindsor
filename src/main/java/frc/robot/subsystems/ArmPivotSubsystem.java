package frc.robot.subsystems;

//import java.util.Set;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.InvertType;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmPivotSubsystem extends SubsystemBase {

    private final CANSparkMax armPivotMotor = new CANSparkMax(Constants.ArmPivotConstants.kArmPivotMotorPort, MotorType.kBrushless);
    private final RelativeEncoder armPivotEncoder = armPivotMotor.getEncoder();

    public double getEncoderMeters() {
        return (((RelativeEncoder) armPivotEncoder).getPosition());
      }

    public ArmPivotSubsystem () {
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ArmPivotEncoder Value",  getEncoderMeters());
    }

    public void setMotor(double speed) {
        SmartDashboard.putNumber("pivot speed", speed);
        SmartDashboard.putNumber("pivot speed * throttle", speed*Constants.ArmPivotConstants.kArmPivotSpeedPercentage);
        armPivotMotor.set(speed*Constants.ArmPivotConstants.kArmPivotSpeedPercentage);
    }

}