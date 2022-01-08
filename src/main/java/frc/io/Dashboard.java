package frc.io;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auton.mode.AutonMode;
import frc.imaging.SimLimelight;
import frc.robot.RobotConstants;
import frc.subsystems.Indexer;
import frc.util.PIDConstants;
import frc.util.ProfileConstants;
import frc.util.TrajectoryConfig;

//The Dashboard prints everything and every value that is important on to the dashboard
public class Dashboard {
	private static Dashboard instance;
	private boolean manual = false;
	private SensorInput sensorIn;
	private RobotOutput robotOut;

	private Dashboard() {
		this.sensorIn = SensorInput.getInstance();
		this.robotOut = RobotOutput.getInstance();
		SmartDashboard.putNumber("Path Turn P", RobotConstants.PATH_TURN_P);
	}

	public static Dashboard getInstance() {
		if (instance == null) {
			instance = new Dashboard();
		}
		return instance;
	}

	public void updateAll() {
		updateSensorDisplay();
	}

	public void updateSensorDisplay() {

		SmartDashboard.putNumber("123_Gyro", this.sensorIn.getGyroAngle());
		SmartDashboard.putNumber("123_X Position: ", this.sensorIn.getDriveXPos());
		SmartDashboard.putNumber("123_Y Position: ", this.sensorIn.getDriveYPos());
		// SmartDashboard.putNumber("123_Left Encoder: ", this.sensorIn.getEncoderLeft());
		// SmartDashboard.putNumber("123_Right Encoder: ", this.sensorIn.getEncoderRight());
		// SmartDashboard.putNumber("123_Drive Speed FPS: ", this.sensorIn.getDriveSpeedFPS());
		//SmartDashboard.putNumber("123_Pressure_Sensor", this.sensorIn.getPressure());
		//SmartDashboard.putNumber("Vision Target Area", this.sensorIn.getVisionTargetArea());
		// SmartDashboard.putBoolean("Compressor Enabled", this.robotOut.getCompressorState());
		SmartDashboard.putNumber("SHOOTER RPM", this.sensorIn.getShooterRPM());
		// SmartDashboard.putNumber("GORDON RAMSEY Velcoity",
		// this.robotOut.getShooterRawVelocity());
		SmartDashboard.putNumber("TURRET ANGLE", this.sensorIn.getTurretAngle());
		SmartDashboard.putNumber("Turret Encoder Ticks", this.robotOut.getTurretEncoderPosition());
		//SmartDashboard.putNumber("Vision Target Rotation", this.sensorIn.getVisionTargetRotation());
		//SmartDashboard.putNumber("PDPCurrentPort0", this.sensorIn.getCurrent(0));
		//SmartDashboard.putNumber("PDPCurrentPort1", this.sensorIn.getCurrent(1));
		//SmartDashboard.putNumber("PDPCurrentPort2", this.sensorIn.getCurrent(2));
		//SmartDashboard.putNumber("PDPCurrentPort3", this.sensorIn.getCurrent(3));
		//SmartDashboard.putNumber("Master Output", this.robotOut.getMasterOutput());
		// SmartDashboard.putNumber("Indexer raw position", this.robotOut.getRawIndexerSpinnerPosition());
		// SmartDashboard.putNumber("Indexer raw velocity", this.robotOut.getRawIndexerSpinnerVelocity());
		SmartDashboard.putNumber("Indexer rpm", this.sensorIn.getIndexerSpinnerRPM());
		// SmartDashboard.putNumber("Indexer degree position", this.sensorIn.getIndexerPositionDegrees());
		// SmartDashboard.putNumber("Indexer running degree position", this.sensorIn.getRunningIndexerPositionDegrees());
		// SmartDashboard.putNumber("Indexer Elevator raw velocity", this.robotOut.getRawIndexerElevatorVelocity());
		SmartDashboard.putNumber("Indexer Elevator Raw Position", this.robotOut.getRawIndexerElevatorPosition());
		SmartDashboard.putNumber("Indexer Elevator RPM", this.sensorIn.getIndexerElevatorRPM());
		// SmartDashboard.putNumber("LEFT DRIVE ENCODER AVERAGE", this.robotOut.getLeftDriveAveragePosition());
		// SmartDashboard.putNumber("RIGHT DRIVE ENCODER AVERAGE", this.robotOut.getRightDriveAveragePosition());
		// SmartDashboard.putNumber("DRIVE FPS AVERAGE", this.robotOut.getDriveFPSAverage());
		// SmartDashboard.putNumber("DRIVE LEFT RAW ENCODER SPEED",this.robotOut.getDriveL1Velocity());
		// SmartDashboard.putNumber("DRIVE RIGHT RAW ENCODER SPEED",this.robotOut.getDriveR1Velocity());
		SmartDashboard.putNumber("SHOOTER MASTER RAW ENCODER", this.robotOut.getShooterMasterEncoderRaw());
		SmartDashboard.putNumber("SHOOTER MASTER RAW TICKS PER 100 MS", this.robotOut.getShooterMasterTicksPer100ms());
		SmartDashboard.putNumber("HOOD ENCODER", this.sensorIn.getHoodEncoder());
		//SmartDashboard.putNumber("HOOD COUNT", this.sensorIn.getHoodCount());
		//SmartDashboard.putNumber("PRESSURE SENSOR", this.sensorIn.getPressure());
		SmartDashboard.putNumber("LEFT HANGER POSISTION", this.robotOut.getLeftHangerPosition());
		SmartDashboard.putNumber("RIGHT HANGER POSISTION", this.robotOut.getRightHangerPosition());
		SmartDashboard.putNumber("HOOD_ANGLE", this.sensorIn.getHoodAngle());
		//SmartDashboard.putNumber("GRYO DEGREES PER SECOND", this.sensorIn.getGyroDegreesPerSecond());
		SmartDashboard.putNumber("CAMERA DISTANCE FEET: ", this.sensorIn.getVisionDistanceFeet());
		SmartDashboard.putNumber("right hanger percent output", this.robotOut.getRightHangerPercentOutput());
		SmartDashboard.putNumber("left hanger percent output", this.robotOut.getLeftHangerPercentOutput());
		SmartDashboard.putNumber("MATCH TIME: ", this.sensorIn.getMatchTimeLeft());
	}

	public double getConstant(String name, double defaultValue) {
		return SmartDashboard.getNumber(name, defaultValue);
	}

	// Get the PID Constants
	public PIDConstants getPIDConstants(String name, PIDConstants constants) {
		double p = SmartDashboard.getNumber("5_" + name + " - P Value", constants.p);
		double i = SmartDashboard.getNumber("5_" + name + " - I Value", constants.i);
		double d = SmartDashboard.getNumber("5_" + name + " - D Value", constants.d);
		double ff = SmartDashboard.getNumber("5_" + name + " - FF Value", constants.ff);
		double eps = SmartDashboard.getNumber("5_" + name + " - EPS Value", constants.eps);
		return new PIDConstants(p, i, d, ff, eps);
	}

	// Put the PID Constants on the dashboard
	public void putPIDConstants(String name, PIDConstants constants) {
		SmartDashboard.putNumber("5_" + name + " - P Value", constants.p);
		SmartDashboard.putNumber("5_" + name + " - I Value", constants.i);
		SmartDashboard.putNumber("5_" + name + " - D Value", constants.d);
		SmartDashboard.putNumber("5_" + name + " - FF Value", constants.ff);
		SmartDashboard.putNumber("5_" + name + " - EPS Value", constants.eps);
	}

	public ProfileConstants getProfileConstants(String name, ProfileConstants constants) {
		double p = SmartDashboard.getNumber("5_" + name + " - P Value", constants.p);
		double i = SmartDashboard.getNumber("5_" + name + " - I Value", constants.i);
		double d = SmartDashboard.getNumber("5_" + name + " - D Value", constants.d);
		double vFF = SmartDashboard.getNumber("3_" + name + " - vFF Value", constants.vFF);
		double aFF = SmartDashboard.getNumber("3_" + name + " - aFF Value", constants.aFF);
		double dFF = SmartDashboard.getNumber("3_" + name + " - dFF Value", constants.dFF);
		double gFF = SmartDashboard.getNumber("3_" + name + " - gFF Value", constants.gravityFF);
		double posEps = SmartDashboard.getNumber("3_" + name + " - Pos EPS Value", constants.positionEps);
		double velEps = SmartDashboard.getNumber("3_" + name + " - Vel EPS Value", constants.velocityEps);
		return new ProfileConstants(p, i, d, vFF, aFF, dFF, gFF, posEps, velEps);
	}

	// Put all the profile constants on the smart dashboard
	public void putProfileConstants(String name, ProfileConstants constants) {
		SmartDashboard.putNumber("5_" + name + " - P Value", constants.p);
		SmartDashboard.putNumber("5_" + name + " - I Value", constants.i);
		SmartDashboard.putNumber("5_" + name + " - D Value", constants.d);
		SmartDashboard.putNumber("3_" + name + " - vFF Value", constants.vFF);
		SmartDashboard.putNumber("3_" + name + " - aFF Value", constants.aFF);
		SmartDashboard.putNumber("3_" + name + " - dFF Value", constants.dFF);
		SmartDashboard.putNumber("3_" + name + " - gFF Value", constants.gravityFF);
		SmartDashboard.putNumber("3_" + name + " - Pos EPS Value", constants.positionEps);
		SmartDashboard.putNumber("3_" + name + " - Vel EPS Value", constants.velocityEps);
	}

	// Get the acceleration and velocity values
	public TrajectoryConfig getTrajectoryConfig(String name, TrajectoryConfig constants) {
		double maxAccel = SmartDashboard.getNumber("3_" + name + " - Max Accel Value", constants.maxAcceleration);
		double maxDecel = SmartDashboard.getNumber("3_" + name + " - Max Decel Value", constants.maxDeceleration);
		double maxVel = SmartDashboard.getNumber("3_" + name + " - Max Vel Value", constants.maxVelocity);
		return new TrajectoryConfig(maxAccel, maxDecel, maxVel);

	}

	// Get the PID Turn
	public double getPathTurnP() {
		return SmartDashboard.getNumber("Path Turn P", RobotConstants.PATH_TURN_P);
	}

	// Put all the acceleration and velocity values on screen
	public void putTrajectoryConfig(String name, TrajectoryConfig constants) {
		SmartDashboard.putNumber("3_" + name + " - Max Accel Value", constants.maxAcceleration);
		SmartDashboard.putNumber("3_" + name + " - Max Decel Value", constants.maxDeceleration);
		SmartDashboard.putNumber("3_" + name + " - Max Vel Value", constants.maxVelocity);

	}

}
