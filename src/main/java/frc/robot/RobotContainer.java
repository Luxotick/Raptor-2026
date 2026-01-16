// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swerve.Swerve;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.atardover.*;

public class RobotContainer {
  final CommandXboxController joystick = new CommandXboxController(0);

  public final Swerve swerve = Swerve.getInstance();
  // public final Climb climb = Climb.getInstance();

  public final Telemetry logger = new Telemetry();
  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
        joystick.rightTrigger().whileTrue(atarDover.getInstance().runVerici());
        joystick.leftTrigger().whileTrue(atarDover.getInstance().runAlici());
        joystick.leftBumper().whileTrue(atarDover.getInstance().runTersAtarDover());

    SwerveInputStream driveAngularVelocity =
        SwerveInputStream.of(
                swerve.getSwerveDrive(),
                () -> joystick.getLeftY() * -1,
                () -> joystick.getLeftX() * -1)
            .withControllerRotationAxis(() -> -joystick.getRightX())
            .deadband(OperatorConstants.DEADBAND)
            .allianceRelativeControl(true);

    swerve.setDefaultCommand(swerve.driveCommand2(driveAngularVelocity));

    joystick.povUp().onTrue(Commands.runOnce(swerve::zeroGyro));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
