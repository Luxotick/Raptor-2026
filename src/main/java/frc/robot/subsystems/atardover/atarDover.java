// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.atardover;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

public class atarDover extends SubsystemBase {

  private static atarDover instance;

  private final SparkMax atarDover =
      new SparkMax(OuttakeConstants.kAtarDoverMotorID, MotorType.kBrushless);
  private final SparkMax firFirDover =
      new SparkMax(OuttakeConstants.kFirFirDoverMotorID, MotorType.kBrushless);

  public static synchronized atarDover getInstance() {
    if (instance == null) {
      instance = new atarDover();
    }
    return instance;
  }

  private atarDover() {
    configureMotors();
  }

  private void configureMotors() {
    SparkMaxConfig atarDoverConfig = new SparkMaxConfig();
    SparkMaxConfig firFirDoverConfig = new SparkMaxConfig();

    atarDoverConfig
        .smartCurrentLimit(OuttakeConstants.kOuttakeCurrentLimit)
        .inverted(OuttakeConstants.kAtarDoverMotorInverted);
    firFirDoverConfig
        .smartCurrentLimit(OuttakeConstants.kOuttakeCurrentLimit)
        .inverted(OuttakeConstants.kFirFirDoverMotorInverted);

    atarDover.configure(
        atarDoverConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    firFirDover.configure(
        firFirDoverConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
  }

  public void setAliciVoltage(double atarVoltage, double firfirVoltage) {
    atarDover.set(1);
    firFirDover.set(0.7);
  }

  public void setVericiVoltage(double atarVoltage, double firfirVoltage) {
    atarDover.set(1);
    firFirDover.set(-0.7);
  }
  
  public void tersAtardover(){
    atarDover.set(-1);
    firFirDover.set(0.7);
  }



  public void stopAliciVerici() {
    atarDover.set(0.0);
    firFirDover.set(0.0);
  }

  //   public boolean getSensorState() {
  // return mSensor.get();
  //   }

  //   public boolean getKeepOuttaking() {
  // return keepOuttaking;
  //   }

  public Command runAlici() {
    return runEnd(
        () -> {
          // keepOuttaking = true;
          // sensorChecker.calculate(false);
          setAliciVoltage(
              OuttakeConstants.kCoralOuttakeVoltage, OuttakeConstants.kCoralOuttakeVoltage);
        },
        () -> stopAliciVerici());
  }

  public Command runVerici() {
    return runEnd(
        () -> {
          // keepOuttaking = true;
          // sensorChecker.calculate(false);
          setVericiVoltage(
              OuttakeConstants.kCoralIntakeVoltage, OuttakeConstants.kCoralIntakeVoltage);
        },
        () -> stopAliciVerici());
  }

  public Command runTersAtarDover() {
    return runEnd(
        () -> {
          // keepOuttaking = true;
          // sensorChecker.calculate(false);
          tersAtardover();  
        },
        () -> stopAliciVerici());
  }

  // public Command runAlgaeOuttake() {
  //   return startEnd(
  //       () -> {
  //         // keepOuttaking = true;
  //         // sensorChecker.calculate(false);
  //         setCoralVoltage(OuttakeConstants.kAlgaeOuttakeVoltage);
  //       },
  //       () -> stopCoral());
  // }

  // public Command runAlgaeIntake() {
  //   return runEnd(
  //       () -> {
  //         setAlgaeVoltage(OuttakeConstants.kAlgaeIntakeVoltage);
  //       },
  //       () -> holdCoral());
  //   //   .unless(() -> mSensor.get())
  //   //   .until(() -> mSensor.get());
  // }
}
