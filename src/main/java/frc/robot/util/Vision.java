package frc.robot.util;

import limelight.Limelight;
import limelight.networktables.Orientation3d;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightSettings.LEDMode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

import java.util.Optional;

import frc.robot.subsystems.swerve.Swerve;

public class Vision {

    // ONEMLI: Limelight hostname'i ile birebir ayni olmali!
    // Limelight arayuzunde Settings > Networking altinda Hostname kisminda gorursun
    private final String LIMELIGHT_NAME_RIGHT = "limelight"; 
    // private final String LIMELIGHT_NAME_LEFT = "limelight-left"; // Gelecek icin rezerve

    private Limelight limelightRight = null;
    private LimelightPoseEstimator poseEstimatorRight = null;

    private final Swerve swerve;

    private final StructPublisher<Pose2d> visionPosePublisher = NetworkTableInstance.getDefault()
        .getTable("Vision").getStructTopic("Pose", Pose2d.struct).publish();

    public Vision(Swerve swerve) {
        this.swerve = swerve;
        
        // Limelight tanimlamalari
        boolean available = Limelight.isAvailable(LIMELIGHT_NAME_RIGHT);
        SmartDashboard.putBoolean("Vision/LimelightAvailable", available);

        if (!available) {
            // Camera not available on NT yet — skip creating objects to avoid warnings and NT subscriber overload.
            return;
        }

        this.limelightRight = new Limelight(LIMELIGHT_NAME_RIGHT);
        // Pose estimator'i burada BIR KERE olusturuyoruz. Surekli olusturmak NT hatasi verdirir.
        this.poseEstimatorRight = limelightRight.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG2);

        // Ayarlarin yapilmasi
        configureLimelight(limelightRight, 0.25, -0.25, 0.25); // Sagdaki kamera icin ornek offset (X, -Y, Z)
    }

    private void configureLimelight(Limelight ll, double forward, double right, double up) {
         // Robot merkezine göre kameranın konumu (Metre cinsinden)
         // Bu değerleri robotunuza göre DÜZENLEMENİZ GEREKİR.
         
         ll.getSettings()
            .withLimelightLEDMode(LEDMode.PipelineControl)
            .withCameraOffset(new Pose3d(forward, right, up, new Rotation3d(0, 0, 0))) 
            .save();
    }

    public void updatePoseEstimation() {
        if (limelightRight == null || poseEstimatorRight == null) {
           // If camera/estimator not ready yet, report and skip.
           SmartDashboard.putBoolean("Vision/EstimatorReady", false);
           return;
        }
        SmartDashboard.putBoolean("Vision/EstimatorReady", true);
        updatePoseForLimelight(limelightRight, poseEstimatorRight);
    }

    private void updatePoseForLimelight(Limelight ll, LimelightPoseEstimator estimator) {
        // MegaTag2 için Robot Orientation güncellenmesi gerekiyor
        
        // Swerve'den gyro verilerini alıyoruz
        Rotation2d heading = swerve.getHeading();
        ChassisSpeeds robotVelocity = swerve.getVelocity();
        
        // Orientation3d oluşturma
        // Pitch ve Roll için elimizde veri yoksa 0 kabul ediyoruz.
        // Yaw (başlık açısı) swerve'den geliyor.
        // Açısal hızlar (Angular Velocity) MegaTag2 için önemlidir. Yaw hızı swerve'den gelir.
        
        ll.getSettings()
          .withRobotOrientation(
              new Orientation3d(
                  new Rotation3d(0, 0, heading.getRadians()), 
                  new AngularVelocity3d(
                      edu.wpi.first.units.Units.DegreesPerSecond.of(0), // Pitch velocity
                      edu.wpi.first.units.Units.DegreesPerSecond.of(0), // Roll velocity
                      edu.wpi.first.units.Units.DegreesPerSecond.of(Units.radiansToDegrees(robotVelocity.omegaRadiansPerSecond)) // Yaw velocity
                  )
              )
          )
          .save();
        
        // Debugging info - Limelight JSON results kontrol
        var results = ll.getLatestResults();
        boolean hasJsonResults = results.isPresent();
        SmartDashboard.putBoolean("Vision/HasJsonResults", hasJsonResults);
        
        if (hasJsonResults) {
            var r = results.get();
            SmartDashboard.putNumber("Vision/FiducialCount", r.targets_Fiducials != null ? r.targets_Fiducials.length : 0);
        }

    // Pose Estimate alma (MegaTag2) - use alliance-aware estimate when possible
    Optional<PoseEstimate> visionEstimate = estimator.getAlliancePoseEstimate();
        
    SmartDashboard.putBoolean("Vision/HasPose", visionEstimate.isPresent());
        
        if (visionEstimate.isPresent()) {
            var pe = visionEstimate.get();
            SmartDashboard.putBoolean("Vision/PoseHasData", pe.hasData);
            SmartDashboard.putNumber("Vision/TagCount", pe.tagCount);
            SmartDashboard.putNumber("Vision/AvgTagDist", pe.avgTagDist);
            SmartDashboard.putString("Vision/PoseX", String.format("%.2f", pe.pose.getX()));
            SmartDashboard.putString("Vision/PoseY", String.format("%.2f", pe.pose.getY()));
        }

        visionEstimate.ifPresent(poseEstimate -> {
            // Sadece veri varsa ekle
            if (poseEstimate.hasData && poseEstimate.tagCount > 0) {
                // Pose verisini swerve odometrysine ekle
                swerve.addVisionMeasurement(
                    poseEstimate.pose.toPose2d(), 
                    poseEstimate.timestampSeconds,
                    null // Standart sapmalar varsayilan
                );
                
                // AdvantageScope icin vision pose yayinla
                visionPosePublisher.set(poseEstimate.pose.toPose2d());
            }
        });
    }
}
