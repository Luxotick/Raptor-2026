package frc.robot.util;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.Swerve;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

/**
 * Vision subsystem for AprilTag-based robot localization using Limelight cameras.
 * Uses YALL (Yet Another Limelight Library) for MegaTag2 pose estimation.
 */
public class Vision {

    // ==================== CONFIGURATION ====================
    
    /** Limelight network table name - must match the hostname in Limelight settings */
    private static final String LIMELIGHT_NAME = "limelight-right";
    
    /** Camera position relative to robot center (meters) - ADJUST THESE VALUES */
    private static final Pose3d CAMERA_OFFSET = new Pose3d(
        0.25,   // X: Forward (+) / Backward (-)
        -0.25,  // Y: Left (+) / Right (-)
        0.25,   // Z: Up (+) / Down (-)
        new Rotation3d(0, 0, 0)  // Roll, Pitch, Yaw (radians)
    );
    
    /** Maximum tag distance for valid measurements (meters) */
    private static final double MAX_TAG_DISTANCE = 4.0;
    
    /** Standard deviation configuration for pose estimation */
    private static final class StdDevConfig {
        static final double BASE_XY = 0.7;      // Base XY standard deviation (meters)
        static final double BASE_THETA = 0.9;  // Base theta standard deviation (radians)
        static final double MIN_XY = 0.5;
        static final double MAX_XY = 5.0;
        static final double MIN_THETA = 0.3;
        static final double MAX_THETA = 3.0;
        static final double DISTANCE_FACTOR = 0.5;    // StdDev increase per meter
        static final double SPEED_FACTOR = 0.8;       // StdDev increase per m/s
        static final double SINGLE_TAG_PENALTY = 1.5; // Multiplier for single tag
    }

    // ==================== INSTANCE VARIABLES ====================
    
    private final Swerve swerve;
    private Limelight limelight;
    private LimelightPoseEstimator poseEstimator;
    private boolean isInitialized = false;
    
    private final StructPublisher<Pose2d> visionPosePublisher;

    // ==================== CONSTRUCTOR ====================
    
    /**
     * Creates a new Vision subsystem.
     * 
     * @param swerve The swerve drive subsystem for odometry integration
     */
    public Vision(Swerve swerve) {
        this.swerve = swerve;
        
        // Setup NetworkTables publisher for AdvantageScope visualization
        this.visionPosePublisher = NetworkTableInstance.getDefault()
            .getTable("Vision")
            .getStructTopic("VisionPose2d", Pose2d.struct)
            .publish();
        
        initializeLimelight();
    }

    // ==================== INITIALIZATION ====================
    
    /**
     * Initializes the Limelight camera and pose estimator.
     */
    private void initializeLimelight() {
        boolean available = Limelight.isAvailable(LIMELIGHT_NAME);
        SmartDashboard.putBoolean("Vision/LimelightAvailable", available);
        
        if (!available) {
            SmartDashboard.putString("Vision/Status", "Limelight not found");
            return;
        }
        
        try {
            limelight = new Limelight(LIMELIGHT_NAME);
            poseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
            
            // Configure Limelight settings
            limelight.getSettings()
                .withLimelightLEDMode(LEDMode.PipelineControl)
                .withImuMode(ImuMode.ExternalImu)
                .withCameraOffset(CAMERA_OFFSET)
                .save();
            
            isInitialized = true;
            SmartDashboard.putString("Vision/Status", "Initialized");
        } catch (Exception e) {
            SmartDashboard.putString("Vision/Status", "Init failed: " + e.getMessage());
        }
    }

    // ==================== PERIODIC UPDATE ====================
    
    /**
     * Updates the pose estimation from vision data.
     * Call this method periodically (e.g., in Robot.periodic() or a subsystem periodic).
     */
    public void updatePoseEstimation() {
        SmartDashboard.putBoolean("Vision/Initialized", isInitialized);
        
        if (!isInitialized) {
            return;
        }
        
        updateRobotOrientation();
        processVisionMeasurement();
    }

    /**
     * Updates the Limelight with current robot orientation.
     * Required for MegaTag2 pose estimation.
     * Uses raw gyro yaw instead of odometry heading for more accurate vision pose.
     */
    private void updateRobotOrientation() {
        ChassisSpeeds velocity = swerve.getVelocity();
        double yawRateDegPerSec = Units.radiansToDegrees(velocity.omegaRadiansPerSecond);
        
        // Use raw gyro yaw instead of odometry heading
        Rotation3d gyroRotation = swerve.getGyroRotation3d();
        double gyroYawDegrees = swerve.getGyroYaw().getDegrees();
        double gyroYawRadians = gyroRotation.getZ();
        
        // Also get odometry heading for comparison
        double odometryHeadingDeg = swerve.getHeading().getDegrees();
        
        // Debug: Log both values to compare
        SmartDashboard.putNumber("Vision/GyroYawDeg", gyroYawDegrees);
        SmartDashboard.putNumber("Vision/GyroYawRad", gyroYawRadians);
        SmartDashboard.putNumber("Vision/OdometryHeadingDeg", odometryHeadingDeg);
        SmartDashboard.putNumber("Vision/YawRateDegPerSec", yawRateDegPerSec);
        
        limelight.getSettings()
            .withRobotOrientation(new Orientation3d(
                gyroRotation,
                new AngularVelocity3d(
                    DegreesPerSecond.of(0),           // Pitch rate
                    DegreesPerSecond.of(0),           // Roll rate
                    DegreesPerSecond.of(yawRateDegPerSec)  // Yaw rate
                )
            ))
            .save();
    }

    /**
     * Processes vision measurements and adds them to the odometry if valid.
     */
    private void processVisionMeasurement() {
        Optional<PoseEstimate> estimate = poseEstimator.getPoseEstimate();
        
        // Update telemetry (this also publishes to AdvantageScope)
        updateTelemetry(estimate);
        
        if (estimate.isEmpty()) {
            SmartDashboard.putString("Vision/RejectReason", "No estimate");
            return;
        }
        
        PoseEstimate pose = estimate.get();
        
        // Validate pose estimate
        if (!isValidPoseEstimate(pose)) {
            return;
        }
        
        // Calculate dynamic standard deviations
        double[] stdDevs = calculateStdDevs(pose);
        
        // Add vision measurement to odometry
        swerve.addVisionMeasurement(
            pose.pose.toPose2d(),
            pose.timestampSeconds,
            VecBuilder.fill(stdDevs[0], stdDevs[0], stdDevs[1])
        );
        
        SmartDashboard.putNumber("Vision/StdDevXY", stdDevs[0]);
        SmartDashboard.putNumber("Vision/StdDevTheta", stdDevs[1]);
        SmartDashboard.putString("Vision/RejectReason", "Accepted");
    }

    // ==================== VALIDATION ====================
    
    /**
     * Validates whether a pose estimate is reliable enough to use.
     * 
     * @param pose The pose estimate to validate
     * @return true if the pose is valid, false otherwise
     */
    private boolean isValidPoseEstimate(PoseEstimate pose) {
        // Must have data
        if (!pose.hasData) {
            return false;
        }
        
        // Must see at least one tag
        if (pose.tagCount <= 0) {
            return false;
        }
        
        // Reject if tags are too far
        if (pose.avgTagDist > MAX_TAG_DISTANCE) {
            SmartDashboard.putString("Vision/RejectReason", "Tags too far");
            return false;
        }
        
        SmartDashboard.putString("Vision/RejectReason", "None");
        return true;
    }

    // ==================== STANDARD DEVIATION ====================
    
    /**
     * Calculates dynamic standard deviations based on measurement quality.
     * Higher standard deviations = less trust in vision measurements.
     * 
     * @param pose The pose estimate
     * @return Array of [xyStdDev, thetaStdDev]
     */
    private double[] calculateStdDevs(PoseEstimate pose) {
        ChassisSpeeds velocity = swerve.getVelocity();
        double speed = Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
        
        // Distance factor: trust decreases with distance
        double distFactor = 1.0 + (pose.avgTagDist * StdDevConfig.DISTANCE_FACTOR);
        
        // Movement factor: trust decreases when moving fast
        double moveFactor = 1.0 + (speed * StdDevConfig.SPEED_FACTOR);
        
        // Tag count factor: single tag is less reliable
        double tagFactor = (pose.tagCount == 1) ? StdDevConfig.SINGLE_TAG_PENALTY : 1.0;
        
        // Calculate final standard deviations
        double xyStdDev = StdDevConfig.BASE_XY * distFactor * moveFactor * tagFactor;
        double thetaStdDev = StdDevConfig.BASE_THETA * distFactor * moveFactor * tagFactor;
        
        // Clamp to configured bounds
        xyStdDev = clamp(xyStdDev, StdDevConfig.MIN_XY, StdDevConfig.MAX_XY);
        thetaStdDev = clamp(thetaStdDev, StdDevConfig.MIN_THETA, StdDevConfig.MAX_THETA);
        
        return new double[] { xyStdDev, thetaStdDev };
    }

    // ==================== TELEMETRY ====================
    
    /**
     * Updates SmartDashboard telemetry with vision data.
     * 
     * @param estimate The current pose estimate
     */
    private void updateTelemetry(Optional<PoseEstimate> estimate) {
        // Target data
        boolean hasTarget = limelight.getData().targetData.getTargetStatus();
        double tagId = limelight.getData().targetData.getAprilTagID();
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget);
        SmartDashboard.putNumber("Vision/TargetID", tagId);
        
        // Pose estimate data
        SmartDashboard.putBoolean("Vision/HasPose", estimate.isPresent());
        
        if (estimate.isPresent()) {
            PoseEstimate pose = estimate.get();
            SmartDashboard.putBoolean("Vision/PoseHasData", pose.hasData);
            SmartDashboard.putNumber("Vision/TagCount", pose.tagCount);
            SmartDashboard.putNumber("Vision/AvgTagDist", pose.avgTagDist);
            SmartDashboard.putNumber("Vision/Timestamp", pose.timestampSeconds);
            
            // Pose2d değerlerini kullan (toPose2d() ile dönüştürülmüş)
            Pose2d pose2d = pose.pose.toPose2d();
            SmartDashboard.putNumber("Vision/PoseX", pose2d.getX());
            SmartDashboard.putNumber("Vision/PoseY", pose2d.getY());
            SmartDashboard.putNumber("Vision/PoseYaw", pose2d.getRotation().getDegrees());
            
            // Pose3d raw değerleri de göster (debug için)
            SmartDashboard.putNumber("Vision/Pose3dX", pose.pose.getX());
            SmartDashboard.putNumber("Vision/Pose3dY", pose.pose.getY());
            SmartDashboard.putNumber("Vision/Pose3dZ", pose.pose.getZ());
            
            // Her zaman visionPosePublisher'ı güncelle
            visionPosePublisher.set(pose2d);
        } else {
            SmartDashboard.putBoolean("Vision/PoseHasData", false);
            SmartDashboard.putNumber("Vision/TagCount", 0);
        }
    }

    // ==================== UTILITY ====================
    
    /**
     * Clamps a value between min and max.
     */
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Returns whether the vision system is initialized and ready.
     * 
     * @return true if initialized
     */
    public boolean isInitialized() {
        return isInitialized;
    }

    /**
     * Gets the Limelight instance for advanced usage.
     * 
     * @return The Limelight instance, or null if not initialized
     */
    public Limelight getLimelight() {
        return limelight;
    }
}
