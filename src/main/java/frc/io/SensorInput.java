package frc.io;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.imaging.SimLimelight;
import frc.imaging.SimLimelightTarget;
import frc.imaging.SimLimelight.LimelightTargetType;
import frc.robot.CoordinateConstants;
import frc.robot.RobotConstants;
import frc.util.RigidTransform2d;
import frc.util.SimAnalogEncoder;
import frc.util.SimEncoder;
import frc.util.SimNavx;
import frc.util.TransformUtil;
import frc.util.Translation2d;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class SensorInput {

	private static SensorInput instance;

	private RobotOutput robotOut;

	private DriverStation driverStation;
	private PowerDistributionPanel pdp;
	private SimNavx navx;
	// Change the I2C port below to match the connection of your color sensor
	//private final I2C.Port i2cPort = I2C.Port.kOnboard;
	//private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
	//private final ColorMatch m_colorMatcher = new ColorMatch();

	private double lastTime = 0.0;
	private double deltaTime = 10.0;

	private boolean firstCycle = true;

	private DriverStationState driverStationMode = DriverStationState.DISABLED;
	private DriverStation.Alliance alliance;
	private double matchTime;

	private long SystemTimeAtAutonStart = 0;
	private long timeSinceAutonStarted = 0;

	private boolean usingNavX = true;

	private double lastLeftDriveSpeedFPS = 0;
	private double lastRightDriveSpeedFPS = 0;

	private double xPosition = 0;
	private double yPosition = 0;
	private double autoStartAngle = 90;

	private double drivePositionState = 0;
	private double driveAccelerationState = 0;
	private double driveVelocityState = 0;
	private double gyroPositionState = 0;
	private double gyroAngle;
	private double lastGyroAngle;

	private SimAnalogEncoder hoodEncoder;

	private SimLimelight limelight;

	private AnalogInput pressureSensor;

	// private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
	// private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
	// private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
	// private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
	// private String colourString;// this is here so we can get this value and give it to spinner class

	private TransformUtil transformUtil = new TransformUtil();

	private SensorInput() {
		this.robotOut = RobotOutput.getInstance();
		this.pdp = new PowerDistributionPanel();
		this.driverStation = DriverStation.getInstance();
		this.navx = new SimNavx();
		this.pressureSensor = new AnalogInput(1);
		this.hoodEncoder = new SimAnalogEncoder(0);
		this.limelight = SimLimelight.getInstance();
		// m_colorMatcher.addColorMatch(kBlueTarget);
		// m_colorMatcher.addColorMatch(kGreenTarget);
		// m_colorMatcher.addColorMatch(kRedTarget);
		// m_colorMatcher.addColorMatch(kYellowTarget);
		//SmartDashboard.putNumber("Gyro Angle", 90);
		transformUtil = TransformUtil.getInstance();

		this.hoodEncoder.zero();
		
		if (this.usingNavX) {
			this.navx = new SimNavx();
		}

		this.reset();
	}

	public static SensorInput getInstance() {
		if (instance == null) {
			instance = new SensorInput();
		}
		return instance;
	}

	public void reset() {
		this.firstCycle = true;
		this.navx.reset();
		if (this.usingNavX) {
			this.navx.reset();
		}

		this.robotOut.resetDriveEncoders();
		
		this.robotOut.resetIndexerEncoder();

		this.robotOut.resetClimbEncoders();

		this.robotOut.resetTurretEncoder();
		

		this.xPosition = 0;
		this.yPosition = 0;
	}

	public void update() {

		
		if (this.lastTime == 0.0) {
			this.deltaTime = 10;
			this.lastTime = System.currentTimeMillis();
		} else {
			this.deltaTime = System.currentTimeMillis() - this.lastTime;
			this.lastTime = System.currentTimeMillis();
		}

		if (this.driverStation.isAutonomous()) {
			this.timeSinceAutonStarted = System.currentTimeMillis() - this.SystemTimeAtAutonStart;
			SmartDashboard.putNumber("12_Time Since Auto Start:", this.timeSinceAutonStarted);
		}

		if (this.firstCycle) {
			this.firstCycle = false;
			if (this.usingNavX) {
				this.lastGyroAngle = this.navx.getAngle() + (this.autoStartAngle - 90);
			} else {
				this.lastGyroAngle = 0.0;
			}
			this.driveVelocityState = 0;
		} else {
			this.driveVelocityState = this.robotOut.getDriveFPSAverage();
		}

		if (this.usingNavX) {
			this.navx.update();
		}

		this.hoodEncoder.update();

		this.alliance = this.driverStation.getAlliance();
		this.matchTime = this.driverStation.getMatchTime();

		if (this.driverStation.isDisabled()) {
			this.driverStationMode = DriverStationState.DISABLED;
		} else if (this.driverStation.isAutonomous()) {
			this.driverStationMode = DriverStationState.AUTONOMOUS;
		} else if (this.driverStation.isOperatorControl()) {
			this.driverStationMode = DriverStationState.TELEOP;
		}

		if (this.usingNavX) {
			this.gyroAngle = this.navx.getAngle() + (this.autoStartAngle - 90);
		} else {
			this.gyroAngle = 0.0;
		}

		this.gyroPositionState = this.gyroAngle;

		double driveXSpeed = this.driveVelocityState * Math.cos(Math.toRadians(this.gyroPositionState));
		double driveYSpeed = this.driveVelocityState * Math.sin(Math.toRadians(this.gyroPositionState));
		this.xPosition += driveXSpeed * this.deltaTime / 1000.0;
		this.yPosition += driveYSpeed * this.deltaTime / 1000.0;

		// Colour sensor stuff below
		// final Color detectedColor = m_colorSensor.getColor();

		// String colorString;
		// final ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

		// if (match.color == kBlueTarget) {
		// 	colorString = "Blue";
		// } else if (match.color == kRedTarget) {
		// 	colorString = "Red";
		// } else if (match.color == kGreenTarget) {
		// 	colorString = "Green";
		// } else if (match.color == kYellowTarget) {
		// 	colorString = "Yellow";
		// } else {
		// 	colorString = "Unknown";
		// }

		// SmartDashboard.putNumber("Red", detectedColor.red);
		// SmartDashboard.putNumber("Green", detectedColor.green);
		// SmartDashboard.putNumber("Blue", detectedColor.blue);
		// SmartDashboard.putNumber("Confidence", match.confidence);
		// SmartDashboard.putString("Detected Color", colorString);

		// this.colourString = colorString;
	}

	public double getMatchTimeLeft() {
		return this.matchTime;
	}

	public DriverStationState getDriverStationMode() {
		return this.driverStationMode;
	}

	public void resetAutonTimer() {
		this.SystemTimeAtAutonStart = System.currentTimeMillis();
	}

	public void setAutoStartAngle(double angle) {
		this.autoStartAngle = angle;
	}

	public double getIndexerSpinnerRPM() {
		return robotOut.getRawIndexerSpinnerVelocity() * 10.0 * 60.0 / RobotConstants.INDEXER_TICKS_PER_REV;
	}

	public double getIndexerPositionDegrees() {
		return (robotOut.getRawIndexerSpinnerPosition() / (RobotConstants.INDEXER_TICKS_PER_REV / 360.0)) % 360.0; // ticks / ticks
																										// per degree
	}

	public double getRunningIndexerPositionDegrees() {
		return robotOut.getRawIndexerSpinnerPosition() / (RobotConstants.INDEXER_TICKS_PER_REV / 360.0);
	}

	public double getIndexerElevatorRPM(){
		return this.robotOut.getRawIndexerElevatorVelocity() / RobotConstants.INDEXER_ELEVATOR_TICKS_PER_REV * 600.0;
	}

	public void setDriveXPos(double x) {
		this.xPosition = x;
	}

	public void setDriveYPos(double y) {
		this.yPosition = y;
	}

	public double getDriveXPos() {
		return this.xPosition;
	}

	public double getDriveYPos() {
		return this.yPosition;
	}

	public double getGyroPositionState() {
		return this.gyroPositionState;
	}

	public double getAngle() {
		return this.gyroAngle;
	}

	public double getDrivePositionState() {
		return this.drivePositionState;
	}

	public double getDriveVelocityState() {
		return this.driveVelocityState;
	}

	public double getDriveAccelerationState() {
		return this.driveAccelerationState;
	}

	public double getDeltaTime() {
		return this.deltaTime;
	}

	public enum DriverStationState {
		AUTONOMOUS, TELEOP, DISABLED,
	}

	public DriverStation.Alliance getAllianceColour() {
		return this.alliance;
	}

	public double getMatchTime() {
		return this.matchTime;
	}

	public long getTimeSinceAutoStarted() {
		return this.timeSinceAutonStarted;
	}

	// PDP //

	public double getVoltage() {
		return this.pdp.getVoltage();
	}

	public double getCurrent(int port) {
		return this.pdp.getCurrent(port);
	}

	// NAVX //
	public double getGyroAngle() {
		if (this.usingNavX) {
			return this.gyroAngle;
		} else {
			return 0;
		}

	}

	public double getGyroDegreesPerSecond(){
		double delta = getGyroAngle() - this.lastGyroAngle;
		double speed = (delta / this.deltaTime) * 100;
		return speed;
	}

	public double getTurretAngle() {
		return this.robotOut.getTurretEncoderPosition() / RobotConstants.TURRET_TICKS_PER_DEGREE;
	}

	public double getPressure() {
		return 250 * (pressureSensor.getVoltage() / 5.0) - 25;
	}

	public double getHoodEncoder(){
		return this.hoodEncoder.get();
	}

	public int getHoodCount(){
		return this.hoodEncoder.getCount();
	}

	public double getHoodAngle(){
		return this.hoodEncoder.get() * RobotConstants.HOOD_DEGREES_PER_TICK;
	}

	public double getShooterRPM(){
		return this.robotOut.getShooterMasterTicksPer100ms() / RobotConstants.SHOOTER_TICKS_PER_REV * 10 * 60;
	}

	// LIMELIGHT
	public boolean getVisionTargetExists() {
		return this.limelight.getTargetExists();
	}

	public void setPipeline(int pipeline){
		this.limelight.setPipeline(pipeline);
	}

	public double getVisionTargetAngle() {
		return this.limelight.getVisionTargetAngle();
	}

	public void setLimelightTarget(LimelightTargetType type) {
		this.limelight.setLimelight(type);
	}

	public LimelightTargetType getLimelightTarget() {
		return this.limelight.getLimelightState();
	}

	public double getVisionTargetX() {
		return this.limelight.getTargetX();
	}

	public double getVisionTargetRotation() {
		return this.limelight.getTargetRotation();
	}

	public double getVisionTargetY() {
		return this.limelight.getTargetY();
	}

	public double getVisionTargetArea() {
		return this.limelight.getTargetArea();
	}

	public double getVisionDistanceInches(){
		//d = (h2-h1) / tan(a1+a2)
		if(this.limelight.getTargetExists()){
			return (RobotConstants.GOAL_HEIGHT_INCHES - RobotConstants.CAMERA_HEIGHT_INCHES) / (Math.tan(Math.toRadians(RobotConstants.CAMERA_TILT_DEGREES + this.limelight.getTargetY())));
		} else {
			return 40.0;
		}
	}
		

	public double getVisionDistanceFeet(){
		return getVisionDistanceInches() / 12.0;
	}

	public double getDriveAngleBasedOnVision() {
		double driveAngle = 0;
		if (this.getVisionTargetExists()) {
			driveAngle = this.getVisionTargetX() - this.getTurretAngle();
		}
		return driveAngle;
	}

	// COLOUR SENSOR
	public String getDetectedColourString() {
		return "no colour sensor bruh";//this.colourString;
	}

	public String getPositionColour() {
		return DriverStation.getInstance().getGameSpecificMessage();
	}
}