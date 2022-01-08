package frc.robot;

import frc.io.Dashboard;
import frc.util.PIDConstants;

public class RobotConstants {

	public static final boolean USING_DASHBOARD = false;
	public static final boolean IS_COMPBOT = true;

	// Drive Size Constants (Feet)
	public static final double DRIVE_MAX_VELOCITY = 14.0;
	public static final double DRIVE_FPS_TO_RPM = (60.0 / (Math.PI * 6.0)) * 12.0;
	public static final double WHEEL_DIAMETER_INCHES = 5.9737;
	public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
	public static final double WHEEL_CIRCUMFERENCE_FEET = WHEEL_CIRCUMFERENCE_INCHES / 12.0;
	public static final double DRIVE_TICKS_PER_REV_HIGH = 2048.0 * (38.0 / 11.0) * (24.0 / 15.0) * (50.0 / 24.0);
	public static final double DRIVE_TICKS_PER_REV_LOW = 2048.0 * (38.0 / 11.0) * (24.0 / 15.0) * (60.0 / 14.0);
	public static final double DRIVE_TICKS_PER_FOOT_HIGH = DRIVE_TICKS_PER_REV_HIGH / WHEEL_CIRCUMFERENCE_FEET;
	public static final double DRIVE_TICKS_PER_FOOT_LOW = DRIVE_TICKS_PER_REV_LOW / WHEEL_CIRCUMFERENCE_FEET;
	public static final double TURRET_TICKS = 4096;
	public static final double TURRET_TICKS_PER_REV = TURRET_TICKS / (18.0/176.0);
	public static final double TURRET_TICKS_PER_DEGREE = TURRET_TICKS_PER_REV / 360.0;
	public static final double TURRET_MAX_ANGLE = 200.0;
	public static final double TURRET_MIN_ANGLE = -31.0;
	public static final double TURRET_DEADZONE_EPS = 0.2;
	public static final double HANGER_TICKS_PER_FOOT = 1;// TODO: do calculation for finding out this number, maybe add
															// a variable for the offset from the ground
	public static final double INDEXER_TICKS_PER_REV = 2048.0 * 36.0;
	public static final double INDEXER_BALL_SLOT_OFFSET = 0.0;
	public static final double INDEXER_ELEVATOR_TICKS_PER_REV = 4096.0;
	public static final double SHOOTER_TICKS_PER_REV = 4096.0;
	public static final double HOOD_DEGREE_DELTA = 45.00;
	public static final double MAX_HOOD_TICKS = 5083;
	public static final double HOOD_DEGREES_PER_TICK = HOOD_DEGREE_DELTA / MAX_HOOD_TICKS;
	public static final double SHOOTER_MAX_RPM = 7000; //change this to reflect our needs/our worries


	//HANGER CONSTANTS 
	
	public static final double HANGER_HIGH_DISTANCE = 150000;
	public static final double HANGER_MID_DISTANCE = 120000;
	public static final double HANGER_LOW_DISTANCE = 60000;
	public static final double HANGER_CLIMB_PRESET = 25000; 


	private static Dashboard dashboard = Dashboard.getInstance();
	private static PIDConstants driveStraightPID = new PIDConstants(2.75, 0.01, 15.0, 0.05);
	private static PIDConstants driveTurnPID = new PIDConstants(0.135, 0.005, 0.85, 1);
	private static PIDConstants gyroPID = new PIDConstants(0, 0, 0, 0);
	private static PIDConstants limeLightTurnPID = new PIDConstants(0.02, 0.000, 0.09, 1);
	private static PIDConstants shooterPID = new PIDConstants(3.0, 0.0015, 50.0, 0.0141, 0);
	private static PIDConstants turretPID = new PIDConstants(1.0, 0.001, 50.0, 50);
	private static PIDConstants hoodPID = new PIDConstants(0.18, 0.0000015, 0, 0.1);
	private static PIDConstants hangerUpPID = new PIDConstants(0.01, 0.0005, 0, 3000);
	private static PIDConstants hangerDownPID = new PIDConstants(0.03, 0.001, 0, 750);
	private static PIDConstants indexerSpinnerPID = new PIDConstants(0, 0, 0, 0.05115, 0.0);
	private static PIDConstants indexerElevatorPID = new PIDConstants(0.05, 0.0, 0.0, 0.075, 0.0);
	private static PIDConstants indexerSpinnerPosPID = new PIDConstants(1.0, 0.0001, 0.0, 5.0);
	private static PIDConstants baselockGyroPositionPID = new PIDConstants(0, 0, 0, 0);
	private static PIDConstants baselockDrivePositionPID = new PIDConstants(0, 0, 0, 0);
	private static PIDConstants driveHighGearVelocityPID = new PIDConstants(0.0, 0, 0, 0.049, 0);
	private static PIDConstants driveLowGearVeloictyPID = new PIDConstants(0, 0, 0, 0, 0);

	public static final double PATH_TURN_P = 15;
	// Camera Constants
	public static final double CAMERA_HORIZONTAL_FOV = 59.6;
	public static final double CAMERA_VERTICAL_FOV = 49.7;
	public static final double CAMERA_IMAGE_WIDTH = 960.0;
	public static final double CAMERA_IMAGE_HEIGHT = 720.0;
	public static final double CAMERA_FOCAL_LENGTH_Y = (CAMERA_IMAGE_HEIGHT / 2)
			/ Math.tan(Math.toRadians(CAMERA_VERTICAL_FOV / 2));
	public static final double CAMERA_FOCAL_LENGTH_X = (CAMERA_IMAGE_WIDTH / 2)
			/ Math.tan(Math.toRadians(CAMERA_HORIZONTAL_FOV / 2));
	public static final double Y_ANGLE_PER_PIXEL = CAMERA_VERTICAL_FOV / CAMERA_IMAGE_HEIGHT;
	public static final double X_ANGLE_PER_PIXEL = CAMERA_HORIZONTAL_FOV / CAMERA_IMAGE_WIDTH;

	public static final double GOAL_HEIGHT_INCHES = 81.25;
	public static final double CAMERA_HEIGHT_INCHES = 23.018;
	public static final double CAMERA_TILT_DEGREES = 32.0;

	// SHOOTER DISTANCE VALUES
	public static final double TRIANGLE_SHOT_HOOD_ANGLE = 14.3;
	public static final double TRIANGLE_SHOT_RPM = 3300;
	public static final double TRIANGLE_SHOT_DISTANCE_FEET = 4.86;

	// All of these return the current PID constants/values
	public static PIDConstants getDriveStraightPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("DRIVE_PID", driveStraightPID);
		} else {
			return driveStraightPID;
		}
	}

	public static PIDConstants getDriveTurnPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("TURN_PID", driveTurnPID);
		} else {
			return driveTurnPID;
		}
	}

	public static PIDConstants getGyroPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("DRIVE_GYRO", gyroPID);
		} else {
			return gyroPID;
		}
	}

	public static PIDConstants getLimeLightTurnPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("LIMELIGHT_TURN_PID", limeLightTurnPID);
		} else {
			return limeLightTurnPID;
		}
	}

	public static PIDConstants getShooterPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("SHOOTER_PID", shooterPID);
		} else {
			return shooterPID;
		}
	}

	public static PIDConstants getTurretPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("TURRET_PID", turretPID);
		} else {
			return turretPID;
		}
	}

	public static PIDConstants getHangerUpPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("HANGER_UP_PID", hangerUpPID);
		} else {
			return hangerUpPID;
		}
	}

	public static PIDConstants getHangerDownPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("HANGER_DOWN_PID", hangerDownPID);
		} else {
			return hangerDownPID;
		}
	}

	public static PIDConstants getHoodPID(){
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("HOOD_PID", hoodPID);
		} else {
			return hoodPID;
		}
	}

	public static PIDConstants getIndexerSpinnerPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("INDEXER_SPINNER_PID", indexerSpinnerPID);
		} else {
			return indexerSpinnerPID;
		}
	}

	public static PIDConstants getIndexerElevatorPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("INDEXER_ELEVATOR_PID", indexerElevatorPID);
		} else {
			return indexerElevatorPID;
		}
	}

	public static PIDConstants getIndexerSpinnerPosPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("INDEXER_SPINNER_POS_PID", indexerSpinnerPosPID);
		} else {
			return indexerSpinnerPosPID;
		}
	}

	public static PIDConstants getBaselockDrivePositionPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("BASELOCK_DRIVE_POSITION_PID", baselockDrivePositionPID);
		} else {
			return baselockDrivePositionPID;
		}
	}

	public static PIDConstants getBaselockGyroPositionPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("BASELOCK_GYRO_POSITION_PID", baselockGyroPositionPID);
		} else {
			return baselockGyroPositionPID;
		}
	}

	public static PIDConstants getDriveHighGearVelocityPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("DRIVE_HIGH_GEAR_VELOCITY_PID", driveHighGearVelocityPID);
		} else {
			return driveHighGearVelocityPID;
		}
	}

	public static PIDConstants getDriveLowGearVelocityPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("DRIVE_LOW_GEAR_VELOCITY_PID", driveLowGearVeloictyPID);
		} else {
			return driveLowGearVeloictyPID;
		}
	}

	// Pushes the values to the smart dashboard

	public static void pushValues() {
		// dashboard.putPIDConstants("DRIVE_PID", driveStraightPID);
		// dashboard.putPIDConstants("TURN_PID", driveTurnPID);
		// dashboard.putPIDConstants("DRIVE_VELOCITY_PID", driveVelocityPID);
		// dashboard.putPIDConstants("LIMELIGHT_TURN_PID", limeLightTurnPID);
		//dashboard.putPIDConstants("SHOOTER_PID", shooterPID);
		// dashboard.putPIDConstants("TURRET_PID", turretPID);
		// dashboard.putPIDConstants("HANGER_UP_PID", hangerUpPID);
		// dashboard.putPIDConstants("HANGER_DOWN_PID", hangerDownPID);
		// dashboard.putPIDConstants("INDEXER_SPINNER_PID", indexerSpinnerPID);
		// dashboard.putPIDConstants("INDEXER_ELEVATOR_PID", indexerElevatorPID);
		// dashboard.putPIDConstants("INDEXER_SPINNER_POS_PID", indexerSpinnerPosPID);
		// dashboard.putPIDConstants("HOOD_PID", hoodPID);

	}
}
