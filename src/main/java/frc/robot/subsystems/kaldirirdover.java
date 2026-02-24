package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class kaldirirdover extends SubsystemBase {
    private final SparkMax kaldirirdover_motor;
    private final int motorId = FuelConstants.KALDIRIR_DOVER;

    public kaldirirdover() {
        kaldirirdover_motor = new SparkMax(motorId, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false);
        config.smartCurrentLimit(40);
        kaldirirdover_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Kaldirirdover Target Position", kaldirirdover_motor.getEncoder().getPosition());
    }

    public void yukari() {
        kaldirirdover_motor.setVoltage(6.0); // Yukarı hareket için pozitif voltaj
    }

    public void asagi() {
        kaldirirdover_motor.setVoltage(-6.0); // Aşağı hareket için negatif voltaj
    }

    public Command yukariCommand() {
    return this.run(() -> yukari());
    }

    public Command asagiCommand() {
        return this.run(() -> asagi());
    }

    public Command stopCommand() {
        return this.runOnce(() -> kaldirirdover_motor.setVoltage(0)).withName("Kaldirirdover Stop");
    }

     public void stop() {
        kaldirirdover_motor.setVoltage(0);
    }

     public double getKaldirirdoverEncoderPosition() {
        return kaldirirdover_motor.getEncoder().getPosition();
    }
}
