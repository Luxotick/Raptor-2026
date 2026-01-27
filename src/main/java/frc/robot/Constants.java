// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static final class FuelConstants {

    public static final int FEEDER_MOTOR_ID = 42;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 41;
    public static final int BICER_DOVER = 43;

    public static final int INDIRIR_DOVER = 44;

    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    public static final double INTAKING_FEEDER_VOLTAGE = -12;
    public static final double INTAKING_INTAKE_VOLTAGE = 10;
    public static final double LAUNCHING_FEEDER_VOLTAGE = 9;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
    public static final double SPIN_UP_FEEDER_VOLTAGE = 6;
    public static final double SPIN_UP_SECONDS = 1;

    //public static final double INTAKING_FEEDER_VOLTAGE = -12;
    //public static final double INTAKING_INTAKE_VOLTAGE = 10;
    //public static final double LAUNCHING_FEEDER_VOLTAGE = 9;
    //public static final double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
    //public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
    //public static final double SPIN_UP_SECONDS = 1;
  }

  public static final double ROBOT_MASS = 51.3;
  public static final double MAX_VELOCITY = 2.0;
  public static final double MAX_ANGULAR_VELOCITY = 120.0;

  public static final class DrivebaseConstants {

    public static final double WHEEL_LOCK_TIME = 10;
  }

  public static class OperatorConstants {

    public static final double DEADBAND = 0.08;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
  }

}
