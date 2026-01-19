// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax intake_motor;
  private final SparkMax launcher_motor;

  public CANFuelSubsystem() {
    launcher_motor = new SparkMax(LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    intake_motor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);

    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_INTAKE_VOLTAGE);

    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
    intake_motor.configure(intakeConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcher_motor.configure(launcherConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
  }

  public void intake() {
    intake_motor.setVoltage(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_INTAKE_VOLTAGE));
    launcher_motor
        .setVoltage(SmartDashboard.getNumber("Intaking intake roller value", INTAKING_LAUNCHER_VOLTAGE));
  }

  public void eject() {
    intake_motor
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_INTAKE_VOLTAGE));
    launcher_motor
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking launcher roller value", INTAKING_LAUNCHER_VOLTAGE));
  }

  public void launch() {
    intake_motor.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_INTAKE_VOLTAGE));
    launcher_motor
        .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  public void stop() {
    intake_motor.set(0);
    launcher_motor.set(0);
  }

  public void spinUp() {
    intake_motor
        .setVoltage(SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_INTAKE_VOLTAGE));
    launcher_motor
        .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  public Command launchCommand() {
    return this.run(() -> launch());
  }

  @Override
  public void periodic() {
  }
}
