package frc.util;

import frc.util.Translation2d;
import frc.util.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CoordinateConstants;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class TransformUtil {

    public static final RigidTransform2d kVehicleToTurretFixed = new RigidTransform2d(
            new Translation2d(CoordinateConstants.kTurretOffsetX, CoordinateConstants.kTurretOffsetY),
            Rotation2d.fromDegrees(CoordinateConstants.kTurretOffsetAngleDegrees));

    public static final RigidTransform2d kTurretRotatingToCamera = new RigidTransform2d(
            new Translation2d(CoordinateConstants.kCameraOffsetX, CoordinateConstants.kCameraOffsetY),
            new Rotation2d());

    public static final RigidTransform2d kCameraToTurretRotated = new RigidTransform2d(
            new Translation2d(-CoordinateConstants.kCameraOffsetX, -CoordinateConstants.kCameraOffsetY),
            new Rotation2d());

    public static final RigidTransform2d kTurretFixedToVehicle = new RigidTransform2d(
            new Translation2d(-CoordinateConstants.kTurretOffsetX, -CoordinateConstants.kTurretOffsetY),
            Rotation2d.fromDegrees(CoordinateConstants.kTurretOffsetAngleDegrees));

    private static TransformUtil instance;
    private Translation2d drivePosition;
    private Rotation2d driveRotation;
    private Rotation2d turretRotation;

    private double[] visionTargetPoints;
    // private double visionTargetY;

    private RigidTransform2d origin;

    public TransformUtil() {
        this.drivePosition = new Translation2d();
        this.driveRotation = new Rotation2d();
        this.turretRotation = new Rotation2d();
        this.visionTargetPoints = new double[8];
        // this.visionTargetY = 0;
    }

    public static TransformUtil getInstance() {
        if (instance == null) {
            instance = new TransformUtil();
        }
        instance.origin = new RigidTransform2d();
        return instance;
    }

    public void updateData(double driveX, double driveY, double driveAngle, double turretAngle, double[] pointsXY) {
        this.drivePosition = new Translation2d(driveX, driveY);
        this.driveRotation = Rotation2d.fromDegrees(-driveAngle);
        this.turretRotation = Rotation2d.fromDegrees(turretAngle);
        this.visionTargetPoints = pointsXY;
        // this.visionTargetY = targetY;
    }

    public RigidTransform2d getDriveTransform() {
        return new RigidTransform2d(this.drivePosition, this.driveRotation);
    }

    public RigidTransform2d getFieldToVehicle() {
        return this.getDriveTransform().transformBy(this.origin);
    }

    public RigidTransform2d getTurretRotation() {
        return new RigidTransform2d(new Translation2d(), this.turretRotation);
    }

    public RigidTransform2d getFieldToTurretRotated() {
        return this.getFieldToVehicle().transformBy(kVehicleToTurretFixed).transformBy(this.getTurretRotation());
    }

    public RigidTransform2d getFieldToCamera() {
        return this.getFieldToTurretRotated().transformBy(kTurretRotatingToCamera);
    }

    public RigidTransform2d getRobotToCamera() {
        return this.getDriveTransform().transformBy(kVehicleToTurretFixed).transformBy(this.getTurretRotation())
                .transformBy(kTurretRotatingToCamera);
    }

    public RigidTransform2d getCameraToTurret() {
        RigidTransform2d cameraToTurret = kCameraToTurretRotated.transformBy(this.turretRotation)
                .transformBy(this.driveRotation.rotateBy(Rotation2d.fromDegrees(90)));

        return cameraToTurret;
    }

    public RigidTransform2d getTurretToRobot() {
        RigidTransform2d cameraToRobot = kTurretFixedToVehicle
                .transformBy(this.driveRotation.rotateBy(Rotation2d.fromDegrees(90)));
        return cameraToRobot;
    }

    public RigidTransform2d getCameraToRobot() {
        return this.getCameraToTurret().transformBy(this.getTurretToRobot());
    }

    public RigidTransform2d getVisionTargetToCamera() {
        double pointOneDistance = getVisionPointDistance(this.visionTargetPoints[0], this.visionTargetPoints[1],
                CoordinateConstants.kCameraPointOneOffsetY);

        double radius = Math.sqrt(Math.pow(visionTargetPoints[0] - (RobotConstants.CAMERA_IMAGE_WIDTH / 2), 2)
                + Math.pow(visionTargetPoints[1] - (RobotConstants.CAMERA_IMAGE_HEIGHT / 2), 2));
        double correctedY = (RobotConstants.CAMERA_IMAGE_HEIGHT / 2)
                + ((visionTargetPoints[1] - (RobotConstants.CAMERA_IMAGE_HEIGHT / 2))
                        / (1 + (0.2968461369307 * Math.pow(radius, 2)) + (-1.43802522547478 * Math.pow(radius, 4))
                                + (-0.00220984214794945 * Math.pow(radius, 6))));

        SmartDashboard.putNumber("Corrected Y Pixel", correctedY);

        return new RigidTransform2d(new Translation2d(), new Rotation2d());
    }

    public double getVisionPointDistance(double pixelX, double pixelY, double realYOffset) {
        double pointYAngle = ((RobotConstants.CAMERA_IMAGE_HEIGHT / 2) - pixelY);
        double pointXAngle = RobotConstants.X_ANGLE_PER_PIXEL * ((RobotConstants.CAMERA_IMAGE_WIDTH / 2) - pixelX);
        double yUp = Math.sqrt(Math.pow(Math.tan(Math.toRadians(pointXAngle)) * RobotConstants.CAMERA_FOCAL_LENGTH_Y, 2)
                - Math.pow((RobotConstants.CAMERA_IMAGE_WIDTH / 2) - pixelX, 2));
        double cameraAngle = Math.atan2(pointYAngle - yUp, RobotConstants.CAMERA_FOCAL_LENGTH_Y);
        double pointDistance = realYOffset / Math.tan(cameraAngle + this.turretRotation.getRadians());
        return pointDistance;
    }

    public RigidTransform2d getVisionTargetToRobot() {
        return this.getVisionTargetToCamera().transformBy(this.getCameraToRobot());
    }
}