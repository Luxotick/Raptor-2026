package frc.robot.util;

import limelight.Limelight;
import limelight.networktables.Orientation3d;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightPoseEstimator.BotPose;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
            .withImuMode(ImuMode.ExternalImu) // Swerve gyro kullaniyoruz, external IMU modu
            .withCameraOffset(new Pose3d(forward, right, up, new Rotation3d(0, 0, 0))) 
            .save();
    }

    public void updatePoseEstimation() {
        if (limelightRight == null || poseEstimatorRight == null) {
           SmartDashboard.putBoolean("Vision/EstimatorReady", false);
           return;
        }
        SmartDashboard.putBoolean("Vision/EstimatorReady", true);
        updatePoseForLimelight(limelightRight, poseEstimatorRight);
    }

    private void updatePoseForLimelight(Limelight ll, LimelightPoseEstimator estimator) {
        Rotation2d heading = swerve.getHeading();
        ChassisSpeeds robotVelocity = swerve.getVelocity();
        
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
        
        var results = ll.getLatestResults();
        boolean hasJsonResults = results.isPresent();
        SmartDashboard.putBoolean("Vision/HasJsonResults", hasJsonResults);
        
        boolean hasTarget = ll.getData().targetData.getTargetStatus();
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget);
        SmartDashboard.putNumber("Vision/TargetID", ll.getData().targetData.getAprilTagID());
        
        if (hasJsonResults) {
            var r = results.get();
            int fiducialCount = (r.targets_Fiducials != null) ? r.targets_Fiducials.length : 0;
            SmartDashboard.putNumber("Vision/FiducialCount", fiducialCount);
            SmartDashboard.putBoolean("Vision/ValidTarget", r.valid);
        }

    boolean isRedAlliance = DriverStation.getAlliance()
        .map(alliance -> alliance == Alliance.Red)
        .orElse(false);
    SmartDashboard.putBoolean("Vision/IsRedAlliance", isRedAlliance);

    Optional<PoseEstimate> visionEstimate = BotPose.BLUE_MEGATAG2.get(limelightRight);
        
    SmartDashboard.putBoolean("Vision/HasPose", visionEstimate.isPresent());
        
        if (visionEstimate.isPresent()) {
            var pe = visionEstimate.get();
            SmartDashboard.putBoolean("Vision/PoseHasData", pe.hasData);
            SmartDashboard.putNumber("Vision/TagCount", pe.tagCount);
            SmartDashboard.putNumber("Vision/AvgTagDist", pe.avgTagDist);
            SmartDashboard.putNumber("Vision/PoseX", pe.pose.getX());
            SmartDashboard.putNumber("Vision/PoseY", pe.pose.getY());
            SmartDashboard.putNumber("Vision/PoseZ", pe.pose.getZ());
        } else {
            SmartDashboard.putBoolean("Vision/PoseHasData", false);
            SmartDashboard.putNumber("Vision/TagCount", 0);
        }

        visionEstimate.ifPresent(poseEstimate -> {
            // Sadece veri varsa ve güvenilir ise ekle
            if (poseEstimate.hasData && poseEstimate.tagCount > 0) {
                
                // Filtreleme - çok uzak ölçümleri reddet
                if (poseEstimate.avgTagDist > 4.0) {
                    return; // Çok uzak, güvenilmez
                }
                
                // Robot hareket halindeyken vision'a daha az güven
                double speed = Math.hypot(swerve.getVelocity().vxMetersPerSecond, 
                                          swerve.getVelocity().vyMetersPerSecond);
                
                // Temel standart sapma - DAHA YÜKSEK değerler = daha az titreme
                // Odometry'ye daha çok güven, vision'a daha az
                double baseXYStdDev = 0.7; // Temel güven (metre) - arttırıldı
                double baseThetaStdDev = 0.9; // Temel açı güveni (radyan) - arttırıldı
                
                // Mesafe faktörü - uzaklaştıkça güven azalır
                double distanceFactor = 1.0 + (poseEstimate.avgTagDist * 0.5);
                
                // Hareket faktörü - hareket halinde güven azalır
                double movementFactor = 1.0 + speed * 0.8;
                
                // Tek tag görüyorsak güveni azalt (daha yüksek stddev)
                double tagCountFactor = (poseEstimate.tagCount == 1) ? 1.5 : 1.0;
                
                double xyStdDev = baseXYStdDev * distanceFactor * movementFactor * tagCountFactor;
                double thetaStdDev = baseThetaStdDev * distanceFactor * movementFactor * tagCountFactor;
                
                // Minimum ve maksimum sınırlar - minimum arttırıldı
                xyStdDev = Math.max(0.5, Math.min(xyStdDev, 5.0));
                thetaStdDev = Math.max(0.3, Math.min(thetaStdDev, 3.0));
                
                SmartDashboard.putNumber("Vision/StdDevXY", xyStdDev);
                
                // Pose verisini swerve odometrysine ekle
                swerve.addVisionMeasurement(
                    poseEstimate.pose.toPose2d(), 
                    poseEstimate.timestampSeconds,
                    edu.wpi.first.math.VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
                );
                
                // AdvantageScope icin vision pose yayinla
                visionPosePublisher.set(poseEstimate.pose.toPose2d());
            }
        });
    }
}
