// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.Swerve;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;
  private final Swerve swerve;

  public Robot() {
    robotContainer = new RobotContainer();
    swerve = Swerve.getInstance();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.logger.periodic();
    robotContainer.vision.updatePoseEstimation();

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Also publish some important encoder/telemetry values from subsystems
    // here so they're visible even when teleop/commands are not actively
    // running. Uses the accessor added in RobotContainer.
    try {
      SmartDashboard.putNumber("Bicerdover Encoder (robotPeriodic)",
          robotContainer.getBicerdoverSubsystem().getBicerdoverEncoderPosition());
      SmartDashboard.putNumber("Indirirdover Encoder (robotPeriodic)",
          robotContainer.getBicerdoverSubsystem().getIndirirdoverEncoderPosition());
    } catch (Exception e) {
      // Don't let telemetry failure stop the scheduler; log if needed.
      // (Avoids exceptions during early init before subsystems are ready.)
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    swerve.setMotorBrake(true);
  }

  @Override
  public void autonomousInit() {
    DataLogManager.log("Starting Auto!");

    // swerve.zeroGyro();

    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
    DataLogManager.log("Starting Teleop!");

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }

  }











  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
