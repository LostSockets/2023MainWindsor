package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.InvertType;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmTelescopicSubsystem extends SubsystemBase {

    private final TalonSRX armTelescopicMotor = new TalonSRX(Constants.ArmTelescopicConstants.kArmTelescopicMotorPort);

    public ArmTelescopicSubsystem () {
    
    }

    @Override
    public void periodic() {
    }

    public void setMotor(double speed) {
        armTelescopicMotor.setNeutralMode(NeutralMode.Brake);

        armTelescopicMotor.set(ControlMode.PercentOutput, speed);
    }

    public double getEncoderMeters() {
        return 0;
    }
}