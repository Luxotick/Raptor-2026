package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class bicerdover extends SubsystemBase {
    private final SparkMax bicerdover_motor = new SparkMax(Constants.FuelConstants.BICER_DOVER, SparkMax.MotorType.kBrushless);
    private final SparkMax indirirdover_motor = new SparkMax(Constants.FuelConstants.INDIRIR_DOVER, SparkMax.MotorType.kBrushless);

    public bicerdover() {
        SmartDashboard.putNumber("Bicerdover Encoder", bicerdover_motor.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Indirirdover Encoder", indirirdover_motor.getAbsoluteEncoder().getPosition());

        SparkMaxConfig bicerdoverConfig = new SparkMaxConfig();
        bicerdoverConfig.smartCurrentLimit(40);
        bicerdover_motor.configure(bicerdoverConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        SparkMaxConfig indirirdoverConfig = new SparkMaxConfig();
        indirirdoverConfig.smartCurrentLimit(40);
        indirirdover_motor.configure(indirirdoverConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }

    public void bicerdover_up() {
        bicerdover_motor.setVoltage(5.0);
    }

    public void bicerdover_down() {
        bicerdover_motor.setVoltage(-5.0);
    }

    public void indirirdover_up() {
        try (PIDController indirirdover_pid = new PIDController(1, 0, 0)) {

            indirirdover_pid.setSetpoint(90);
            while (indirirdover_motor.getAbsoluteEncoder().getPosition() < 90) {
                double output = indirirdover_pid.calculate(indirirdover_motor.getAbsoluteEncoder().getPosition());
                indirirdover_motor.setVoltage(output);
            }

            indirirdover_pid.setSetpoint(0);
            double output = indirirdover_pid.calculate(indirirdover_motor.getAbsoluteEncoder().getPosition());
            indirirdover_motor.setVoltage(output);
        }
    }
    
}
