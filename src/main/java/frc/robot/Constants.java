// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = 51.3;
  // public static final Matter CHASSIS =
  //     new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  // public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_VELOCITY = 2.0;
  public static final double MAX_ANGULAR_VELOCITY = 120.0;

  // Maximum speed of the robot in meters per second, used to limit acceleration.

  //  public static final class AutonConstants
  //  {
  //
  //    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
  //    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  //  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.08;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    // public static final double TURN_CONSTANT = 6;
  }

  public static class VisionConstants {

    public static boolean USE_VISION = true;

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    // public static final double POSE_AMBIGUITY_MULTIPLIER = 3; //4 for auto
    // public static final double NOISY_DISTANCE_METERS = 4; // 2.5 for auto
    // public static final double DISTANCE_WEIGHT = 8; // 10 for auto
    // public static final int TAG_PRESENCE_WEIGHT = 8; // 10 for auto

    public static final double POSE_AMBIGUITY_MULTIPLIER = 4; // 4 for auto
    public static final double NOISY_DISTANCE_METERS = 3.0; // 3 for auto

    public static final double DISTANCE_WEIGHT = 5; // 10 for auto
    public static final int TAG_PRESENCE_WEIGHT = 8; // 10 for auto

    public static final double AUTO_MAX_VISION_DISTANCE = 6.0;
    public static final double AUTO_DISTANCE_WEIGHT = 10;
    public static final int AUTO_TAG_PRESENCE_WEIGHT = 8;

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state
     * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians,
     * then meters.
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS =
        VecBuilder.fill(
            // if these numbers are less than one, multiplying will do bad things
            0.45, // x
            0.45, // y
            999999 // theta
            );

    public static final Matrix<N3, N1> AUTO_VISION_MEASUREMENT_STANDARD_DEVIATIONS =
        VecBuilder.fill(
            // if these numbers are less than one, multiplying will do bad things
            0.55, // x
            0.55, // y
            999999 // theta
            );

    public static final Matrix<N3, N1> YAW_VISION_MEASUREMENT_STANDARD_DEVIATIONS =
        VecBuilder.fill(
            // if these numbers are less than one, multiplying will do bad things
            5.0, // x
            5.0, // y
            5.0 // theta
            );

    public static final Matrix<N3, N1> YAW_VISION_MEASUREMENT_CORRECTION_DEVIATIONS =
        VecBuilder.fill(
            // if these numbers are less than one, multiplying will do bad things
            999999, // x
            999999, // y
            10.0 // theta
            );
  }

  public static final class ElevatorConstants {
    public static final int kLeftMotorID = 62;
    public static final int kRightMotorID = 61;

    public static final double kP = 0.007;
    public static final double kI = 0.0;
    public static final double kD = 0.04;

    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;

    public static final double kMinOutput = 0.0;
    public static final double kMaxOutput = 2500.0;

    public static final double kUnitConversion = 0.0625 * 2 * Math.PI * 2.074 * 25.4 * 2.0;

    public static final double kAllowableError = 25.0;
  }

  public static final class OuttakeConstants {
    public static final int kAtarDoverMotorID = 41;
    public static final int kFirFirDoverMotorID = 42;

    public static final boolean kAtarDoverMotorInverted = false;
    public static final boolean kFirFirDoverMotorInverted = false;

    public static final int kArmMotorID = 58;
    public static final int kCoralMotorID = 57;
    public static final int kCommonMotorID = 55;
    public static final int kAlgaeMotorID = 56;

    public static final boolean kArmMotorInverted = true;
    public static final boolean kCoralMotorInverted = true;
    public static final boolean kCommonMotorInverted = true;
    public static final boolean kAlgaeMotorInverted = false;

    public static final double kArmP = 0.015; // TODO
    public static final double kArmI = 0.0;
    public static final double kArmD = 0.5;

    public static final int kArmCurrentLimit = 30;
    public static final int kOuttakeCurrentLimit = 20;

    public static final double kUnitConversion = 16.0 * 38.0 / 26.0 / 2.0;

    public static final double kArmMinOutput = 0.0;
    public static final double kArmMaxOutput = 100.0;

    public static final double kAllowableError = 5.0;

    public static final double kCoralOuttakeVoltage = 4.0; // TODO
    public static final double kCoralIntakeVoltage = 3.0;
    public static final double kCommonIntakeVoltage = 2.0;

    public static final double kAlgaeOuttakeVoltage = 3.75;
    public static final double kAlgaeIntakeVoltage = 3.8;
  }

  public static final class FieldConstants {
    public static final Pose2d[] kHPBlueTargets = {
      new Pose2d(1.4, 8.05 - 7.28, new Rotation2d(Math.toRadians(54.0))),
      new Pose2d(1.4, 7.28, new Rotation2d(Math.toRadians(-54.0)))
    };

    public static final Pose2d[] kHPRedTargets = {
      new Pose2d(17.55 - 1.4, 8.05 - 7.28, new Rotation2d(Math.toRadians(126.0))),
      new Pose2d(17.55 - 1.4, 7.28, new Rotation2d(Math.toRadians(-126.0)))
    };

    private static final double kVerticalOffset = 0.43;
    private static final double kHorizontalOffset = 0.164;
    private static final Translation2d kLeftTargetOffset =
        new Translation2d(kVerticalOffset, -kHorizontalOffset);
    private static final Translation2d kRightTargetOffset =
        new Translation2d(kVerticalOffset, kHorizontalOffset);
    private static final Translation2d kCenterTargetOffset =
        new Translation2d(kVerticalOffset, 0.0);

    public static final Pose2d[] kReefBlueApriltags = {
      new Pose2d(4.073905999999999, 3.3063179999999996, new Rotation2d(Math.toRadians(240.0))),
      new Pose2d(3.6576, 4.0259, new Rotation2d(Math.toRadians(180.0))),
      new Pose2d(4.073905999999999, 4.745482, new Rotation2d(Math.toRadians(120.0))),
      new Pose2d(4.904739999999999, 4.745482, new Rotation2d(Math.toRadians(60.0))),
      new Pose2d(5.321046, 4.0259, new Rotation2d(Math.toRadians(0.0))),
      new Pose2d(4.904739999999999, 3.3063179999999996, new Rotation2d(Math.toRadians(300.0))),
    };

    public static final Pose2d[] kReefRedApriltags = {
      new Pose2d(13.474446, 3.3063179999999996, new Rotation2d(Math.toRadians(300.0))),
      new Pose2d(13.890498, 4.0259, new Rotation2d(Math.toRadians(0.0))),
      new Pose2d(13.474446, 4.745482, new Rotation2d(Math.toRadians(60.0))),
      new Pose2d(12.643358, 4.745482, new Rotation2d(Math.toRadians(120.0))),
      new Pose2d(12.227305999999999, 4.0259, new Rotation2d(Math.toRadians(180.0))),
      new Pose2d(12.643358, 3.3063179999999996, new Rotation2d(Math.toRadians(240.0)))
    };

    public static final Pose2d[] kReefBlueTargets;
    public static final Pose2d[] kReefRedTargets;

    static {
      int numApriltags = kReefBlueApriltags.length;
      List<Pose2d> blueTargets = new ArrayList<>();
      List<Pose2d> redTargets = new ArrayList<>();

      for (int i = 0; i < numApriltags; i++) {
        blueTargets.add(
            kReefBlueApriltags[i].transformBy(
                new Transform2d(kLeftTargetOffset, new Rotation2d(Math.PI))));
        blueTargets.add(
            kReefBlueApriltags[i].transformBy(
                new Transform2d(kCenterTargetOffset, new Rotation2d(Math.PI))));
        blueTargets.add(
            kReefBlueApriltags[i].transformBy(
                new Transform2d(kRightTargetOffset, new Rotation2d(Math.PI))));
      }

      for (int i = 0; i < numApriltags; i++) {
        redTargets.add(
            kReefRedApriltags[i].transformBy(
                new Transform2d(kLeftTargetOffset, new Rotation2d(Math.PI))));
        redTargets.add(
            kReefRedApriltags[i].transformBy(
                new Transform2d(kCenterTargetOffset, new Rotation2d(Math.PI))));
        redTargets.add(
            kReefRedApriltags[i].transformBy(
                new Transform2d(kRightTargetOffset, new Rotation2d(Math.PI))));
      }

      kReefBlueTargets = blueTargets.toArray(new Pose2d[0]);
      kReefRedTargets = redTargets.toArray(new Pose2d[0]);
    }

    public static final Translation2d kReefBlueCenter =
        new Translation2d(
            (kReefBlueTargets[1].getX() + kReefBlueTargets[10].getX()) / 2.0,
            (kReefBlueTargets[1].getY() + kReefBlueTargets[10].getY()) / 2.0);
    public static final Translation2d kReefRedCenter =
        new Translation2d(
            (kReefRedTargets[1].getX() + kReefRedTargets[10].getX()) / 2.0,
            (kReefRedTargets[1].getY() + kReefRedTargets[10].getY()) / 2.0);
  }
}
