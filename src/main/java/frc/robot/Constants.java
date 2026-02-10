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
    public static final int DONME_DOLAP = 45;

    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    public static final double INTAKING_FEEDER_VOLTAGE = -12;
    public static final double INTAKING_INTAKE_VOLTAGE = 10;
    public static final double LAUNCHING_FEEDER_VOLTAGE = 9;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
    public static final double SPIN_UP_FEEDER_VOLTAGE = 6;
    public static final double SPIN_UP_SECONDS = 1;

    // Bicerdover motor voltajı
    public static final double BICERDOVER_RUN_VOLTAGE = 5.0;
    
    // Donmedolap motor voltajı (yavaş hız)
    public static final double DONMEDOLAP_SLOW_VOLTAGE = 2.0;

    // Indirirdover PID kazançları - güvenli başlangıç değerleri
    // Absolute encoder 0-1 rotasyon döndürür, voltaj çıkışlı PID
    public static final double INDIRIRDOVER_KP = 4.0;   // Konservatif P kazancı
    public static final double INDIRIRDOVER_KI = 0.0;   // I kazancı (başlangıçta 0, gerekirse ekle)
    public static final double INDIRIRDOVER_KD = 0.1;   // D kazancı - osilasyonu azaltır
    public static final double INDIRIRDOVER_MAX_VOLTAGE = 6.0; // Güvenlik sınırı

    // Indirirdover pozisyonları (rotasyon cinsinden, 0-1 arası)
    // !! MEKANİZMANIZA GÖRE AYARLAYIN - encoder değerlerini SmartDashboard'dan okuyun !!
    public static final double INDIRIRDOVER_UP_POSITION = 8;
    public static final double INDIRIRDOVER_DOWN_POSITION = 0.5;

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

  public static class OperatorConstants {

    public static final double DEADBAND = 0.08;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
  }

}
