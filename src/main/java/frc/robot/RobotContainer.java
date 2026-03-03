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
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.bicerdover;
import frc.robot.subsystems.kaldirirdover;
// Vision şimdilik devre dışı
// import frc.robot.util.Vision;

public class RobotContainer {
  final CommandXboxController joystick = new CommandXboxController(0);

  private final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem();
  private final bicerdover bicerdoverSubsystem = new bicerdover();
  //private final kaldirirdover kaldirirdoverSubsystem = new kaldirirdover();

  public final Swerve swerve = Swerve.getInstance();
  // Vision şimdilik devre dışı - NavX2 heading kullanılıyor
  // public final Vision vision = new Vision(swerve);
  // public final Climb climb = Climb.getInstance();

  public final Telemetry logger = new Telemetry();
  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {

    // PathPlanner NamedCommands kayıtları - autoChooser'dan ÖNCE yapılmalı
    // "launchWithDonmedolap": launch + dönme dolabı paralel, 10 saniye süre sınırı
    NamedCommands.registerCommand("launchWithDonmedolap",
        Commands.parallel(
            ballSubsystem.launchCommand(),
            bicerdoverSubsystem.donmedolapSlowCommand()
        ).withTimeout(5)
         .finallyDo(() -> {
            ballSubsystem.stop();
            bicerdoverSubsystem.donmedolapStop();
         })
    );
    /*
    NamedCommands.registerCommand("asilma",
        Commands.parallel(
            kaldirirdoverSubsystem.yukariCommand().withTimeout(3),
            kaldirirdoverSubsystem.asagiCommand().withTimeout(2)
        ).withTimeout(7)
         .finallyDo(() -> {
            kaldirirdoverSubsystem.stop();
         })
    );
   */

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  // Expose the bicerdover subsystem so Robot (or other managers) can read
  // encoder values / state even when commands aren't actively running.
  public bicerdover getBicerdoverSubsystem() {
    return bicerdoverSubsystem;
  }

  private void configureBindings() {

    joystick.leftBumper()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    joystick.rightBumper()
        .whileTrue((ballSubsystem.launchCommand())
            .finallyDo(() -> ballSubsystem.stop()));

    /* 
    joystick.rightTrigger()
        .whileTrue((kaldirirdoverSubsystem.yukariCommand())
            .finallyDo(() -> kaldirirdoverSubsystem.stopCommand()));
            */
    // A tuşu basılı tutulduğunda: indirirdover UP + bicerdover çalışır + intake çalışır
    // Her iki subsystem paralel çalışır, buton bırakıldığında her şey durur
    joystick.a()
        .whileTrue(Commands.parallel(
            bicerdoverSubsystem.fullIntakeCommand(),
            ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop())
        ));

    // B tuşu basılı tutulduğunda: donmedolap (ID 45) yavaş döner
    joystick.b()
        .whileTrue(bicerdoverSubsystem.donmedolapSlowCommand());

    joystick.y()
        .whileTrue(bicerdoverSubsystem.donmedolapReverseCommand());

    SwerveInputStream driveAngularVelocity =
        SwerveInputStream.of(
                swerve.getSwerveDrive(),
                () -> joystick.getLeftY() * -1,
                () -> joystick.getLeftX() * -1)
            .withControllerRotationAxis(() -> -joystick.getRightX())
            .deadband(OperatorConstants.DEADBAND)
            .allianceRelativeControl(true);

    swerve.setDefaultCommand(swerve.driveFieldOriented(driveAngularVelocity));

    joystick.povUp().onTrue(Commands.runOnce(swerve::zeroGyro));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
