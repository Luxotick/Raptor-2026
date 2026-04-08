package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;

public class bicerdover extends SubsystemBase {
    private final SparkMax bicerdover_motor;
    private final SparkMax indirirdover_motor;
    private final SparkMax donmedolap;

    // PID controller - class level field, state korunur
    private final PIDController indirirdoverPID;

    // Indirirdover PID durumu
    private boolean indirirdoverPIDEnabled = false;

    public bicerdover() {
        bicerdover_motor = new SparkMax(FuelConstants.BICER_DOVER, SparkMax.MotorType.kBrushless);
        indirirdover_motor = new SparkMax(FuelConstants.INDIRIR_DOVER, SparkMax.MotorType.kBrushless);
        donmedolap = new SparkMax(FuelConstants.DONME_DOLAP, SparkMax.MotorType.kBrushless);

        // Motor konfigürasyonları
        SparkMaxConfig bicerdoverConfig = new SparkMaxConfig();
        bicerdoverConfig.smartCurrentLimit(40);
        bicerdover_motor.configure(bicerdoverConfig,
                com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters); 

        SparkMaxConfig indirirdoverConfig = new SparkMaxConfig();
        indirirdoverConfig.smartCurrentLimit(40);
        indirirdover_motor.configure(indirirdoverConfig,
                com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);

        // Donmedolap motor konfigürasyonu
        SparkMaxConfig donmedolapConfig = new SparkMaxConfig();
        donmedolapConfig.smartCurrentLimit(40);
        donmedolap.configure(donmedolapConfig,
                com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);

        // PID başlat - güvenli kazançlar
        indirirdoverPID = new PIDController(
                FuelConstants.INDIRIRDOVER_KP,
                FuelConstants.INDIRIRDOVER_KI,
                FuelConstants.INDIRIRDOVER_KD);
        indirirdoverPID.setTolerance(0.02); // 0.02 rotasyon tolerans

        // SmartDashboard'dan PID ayarları yapmak için
        SmartDashboard.putNumber("Indirirdover kP", FuelConstants.INDIRIRDOVER_KP);
        SmartDashboard.putNumber("Indirirdover kI", FuelConstants.INDIRIRDOVER_KI);
        SmartDashboard.putNumber("Indirirdover kD", FuelConstants.INDIRIRDOVER_KD);
        SmartDashboard.putNumber("Indirirdover Up Pos", FuelConstants.INDIRIRDOVER_UP_POSITION);
    }

    public void donmedolapRun(){
        donmedolap.setVoltage(FuelConstants.DONMEDOLAP_SLOW_VOLTAGE);
    }

    public void donmeDolapReverseRun(){
        donmedolap.setVoltage(FuelConstants.DONMEDOLAP_REVERSE_VOLTAGE);
    }


    public void donmedolapStop(){
        donmedolap.setVoltage(0);
    }

    /** B tuşu ile çalışacak - donmedolap yavaş döner */
    public Command donmedolapSlowCommand() {
        return this.runEnd(
                () -> donmedolapRun(),
                () -> donmedolapStop());
    }

    public Command donmedolapReverseCommand() {
        return this.runEnd(
                () -> donmeDolapReverseRun(),
                () -> donmedolapStop());
    }

    // --- Bicerdover (döner mekanizma) ---

    public void bicerdoverRun() {
        bicerdover_motor.setVoltage(FuelConstants.BICERDOVER_RUN_VOLTAGE);
    }

    public void bicerdoverStop() {
        bicerdover_motor.setVoltage(0);
    }

    // --- Indirirdover (pozisyon kontrollü) ---

    /**
     * Indirirdover'ı hedef pozisyona PID ile gönderir.
     * Absolute encoder rotasyon cinsinden (0-1 arası) çalışır.
     */
    public void setIndirirdoverTarget(double position) {
        // SmartDashboard'dan güncel PID değerlerini oku (canlı tuning için)
        indirirdoverPID.setPID(
                SmartDashboard.getNumber("Indirirdover kP", FuelConstants.INDIRIRDOVER_KP),
                SmartDashboard.getNumber("Indirirdover kI", FuelConstants.INDIRIRDOVER_KI),
                SmartDashboard.getNumber("Indirirdover kD", FuelConstants.INDIRIRDOVER_KD));
        indirirdoverPID.setSetpoint(position);
        indirirdoverPIDEnabled = true;
    }

    public void stopIndirirdover() {
        indirirdoverPIDEnabled = false;
        indirirdover_motor.setVoltage(0);
    }

    // Public getters so other parts of the robot (e.g., Robot.robotPeriodic)
    // can read encoder positions even if commands aren't running.
    public double getBicerdoverEncoderPosition() {
        return bicerdover_motor.getEncoder().getPosition();
    }

    public double getIndirirdoverEncoderPosition() {
        return indirirdover_motor.getEncoder().getPosition();
    }

    public void stopAll() {
        bicerdoverStop();
        stopIndirirdover();
    }

    public boolean isIndirirdoverAtTarget() {
        return indirirdoverPID.atSetpoint();
    }

    // --- Command'lar ---

    /**
     * A tuşu ile çalışacak tam intake komutu:
     * indirirdover UP pozisyonuna gider + bicerdover döner.
     * Buton bırakıldığında her şey durur.
     */
    public Command fullIntakeCommand() {
        return this.runEnd(
                () -> {
                    setIndirirdoverTarget(
                            SmartDashboard.getNumber("Indirirdover Up Pos",
                                    FuelConstants.INDIRIRDOVER_UP_POSITION));
                    bicerdoverRun();
                },
                () -> stopAll());
    }

    /** Sadece indirirdover up komutu */
    public Command indirirdoverUpCommand() {
        return this.runEnd(
                () -> setIndirirdoverTarget(FuelConstants.INDIRIRDOVER_UP_POSITION),
                () -> stopIndirirdover());
    }

    public Command indirirdoverDownCommand(){
        return this.runEnd(
                () -> setIndirirdoverTarget(FuelConstants.INDIRIRDOVER_DOWN_POSITION),
                () -> stopIndirirdover());              

    }

    /** Sadece bicerdover çalıştırma komutu */
    public Command bicerdoverRunCommand() {
        return this.runEnd(
                () -> bicerdoverRun(),
                () -> bicerdoverStop());
    }

    @Override
    public void periodic() {
        // PID döngüsü burada çalışır - HER 20ms'de bir, robot'u KİLİTLEMEDEN
        if (indirirdoverPIDEnabled) {
            double currentPosition = indirirdover_motor.getEncoder().getPosition();
            double output = indirirdoverPID.calculate(currentPosition);
            // Çıkışı güvenli aralığa sınırla
            output = MathUtil.clamp(output,
                    -FuelConstants.INDIRIRDOVER_MAX_VOLTAGE,
                    FuelConstants.INDIRIRDOVER_MAX_VOLTAGE);
            indirirdover_motor.setVoltage(output);
        }

        // Telemetri
        SmartDashboard.putNumber("Bicerdover Encoder",
                bicerdover_motor.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Indirirdover Encoder",
                indirirdover_motor.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Indirirdover Target",
                indirirdoverPID.getSetpoint());
        SmartDashboard.putBoolean("Indirirdover At Target",
                isIndirirdoverAtTarget());
    }
}
