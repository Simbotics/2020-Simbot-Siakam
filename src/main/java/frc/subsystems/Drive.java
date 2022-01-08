package frc.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.DriverInput;
import frc.io.RobotOutput;
import frc.io.SensorInput;
import frc.robot.RobotConstants;
import frc.subsystems.LEDStrip.LEDColourState;
import frc.util.SimLib;
import frc.util.SimPID;
import frc.util.SimPIDF;
import frc.util.SimPoint;

public class Drive extends Subsystem {

	public enum DriveState {
		OUTPUT, VELOCITY, BASELOCKED, DRIVING_TO_TRIANGLE
	}

	private static Drive instance;
	private RobotOutput robotOut;
	private SensorInput sensorIn;
	private DriverInput driverIn;
	private DriveState currentState = DriveState.OUTPUT;
	private double leftOut;
	private double rightOut;
	private double yOutput = 0;
	private double xOutput = 0;
	private LEDColourState desiredLEDState = LEDColourState.OFF;
	private boolean firstBaselockLockCycle = false;

	private SimPID straightPID;
	private SimPID turnPID;
	private SimPID limelightTurnPID;
	private SimPID baselockDrivePosPID;
	private SimPID baselockGyroPosPID;

	public static Drive getInstance() {
		if (instance == null) {
			instance = new Drive();
		}
		return instance;
	}

	private Drive() {
		this.robotOut = RobotOutput.getInstance();
		this.sensorIn = SensorInput.getInstance();
		this.driverIn = DriverInput.getInstance();
		this.firstCycle();
	}

	// rotates the xy coordinates to be relative to the angle of the target
	private SimPoint getRotatedError(double theta, double desiredX, double desiredY) {
		double currentX = this.sensorIn.getDriveXPos();
		double currentY = this.sensorIn.getDriveYPos();
		double rotation = 90 - theta;

		SimPoint currentPosition = new SimPoint(currentX, currentY);
		SimPoint finalPosition = new SimPoint(desiredX, desiredY);

		currentPosition.rotateByAngleDegrees(rotation);
		finalPosition.rotateByAngleDegrees(rotation);

		double xError = finalPosition.getX() - currentPosition.getX();
		double yError = finalPosition.getY() - currentPosition.getY();

		return new SimPoint(xError, yError);

	}

	@Override
	public void firstCycle() {
		this.straightPID = new SimPID(RobotConstants.getDriveStraightPID());
		this.straightPID.setIRange(1);
		this.straightPID.setMaxOutput(13);
		this.straightPID.setMinDoneCycles(5);

		this.robotOut.configureHighGearVelPID(RobotConstants.getDriveHighGearVelocityPID());
		this.turnPID = new SimPID(RobotConstants.getDriveTurnPID());
		this.turnPID.setMinDoneCycles(5);
		this.turnPID.setMaxOutput(8);
		this.turnPID.setIRange(5);

		this.limelightTurnPID = new SimPID(RobotConstants.getLimeLightTurnPID());
		this.limelightTurnPID.setMinDoneCycles(10);

		this.baselockDrivePosPID = new SimPID(RobotConstants.getBaselockDrivePositionPID());
		this.baselockGyroPosPID = new SimPID(RobotConstants.getBaselockGyroPositionPID());
	}

	// Sets the motor output for the drive base
	public void setOutput(double y, double turn) {
		this.leftOut = y + turn;
		this.rightOut = y - turn;
	}

	public void setTargetVelocity(double leftOut, double rightOut) {
		this.leftOut = leftOut;
		this.rightOut = rightOut;
		this.currentState = DriveState.VELOCITY;

	}

	public void setVelocityOutput(double leftOut, double rightOut) {
		this.robotOut.setDriveLeftFPS(leftOut);
		this.robotOut.setDriveRightFPS(rightOut);

	}

	public void setRampRate(double rate) {
		this.robotOut.setDriveRampRate(rate);
	}

	@Override
	public void calculate() {
		SmartDashboard.putString("DRIVE_STATE: ", this.currentState.toString());
		if (this.currentState == DriveState.OUTPUT) {
			this.robotOut.setDriveLeft(this.leftOut);
			this.robotOut.setDriveRight(this.rightOut);
			this.firstBaselockLockCycle = false;
		} else if (this.currentState == DriveState.VELOCITY) {
			setVelocityOutput(this.leftOut, this.rightOut);

		}

		else if (this.currentState == currentState.BASELOCKED) {
			if (Math.abs(robotOut.getDriveFPSAverage()) > 0.1 && !firstBaselockLockCycle) {
				this.firstBaselockLockCycle = false;
				robotOut.setDriveLeftFPS(0.0);
				robotOut.setDriveRightFPS(0.0);
			} else {
				if (!firstBaselockLockCycle) {
					double lockAngle = sensorIn.getGyroAngle();
					double lockPosition = robotOut.getDriveAveragePosition();
					baselockDrivePosPID.setDesiredValue(lockPosition);
					baselockGyroPosPID.setDesiredValue(lockAngle);
					this.firstBaselockLockCycle = true;
				}

				double x = baselockGyroPosPID.calcPID(sensorIn.getGyroAngle());
				double y = baselockDrivePosPID.calcPID(robotOut.getDriveAveragePosition());

				double leftOut = SimLib.calcLeftTankDrive(x, y);
				double rightOut = SimLib.calcRightTankDrive(x, y);

				robotOut.setDriveLeft(leftOut);
				robotOut.setDriveLeft(rightOut);
			}
		}

		else if(this.currentState == currentState.DRIVING_TO_TRIANGLE){
			driveInAStrightLineTriangle();
		}

		//SmartDashboard.putString("DRIVE_STATE", this.currentState.toString());
	}

	public void setyOutput(double yOutput) {
		this.yOutput = yOutput;
	}

	public void setxOutput(double xOutput) {
		this.xOutput = xOutput;
	}

	@Override
	public void disable() {
		this.robotOut.setDriveLeft(0.0);
		this.robotOut.setDriveRight(0.0);
	}

	public boolean DriveToPoint(double x, double y, double theta, double minVelocity, double maxVelocity,
			double turnRate, double maxTurn, double eps) {
		this.straightPID.setMinMaxOutput(minVelocity, maxVelocity);
		SimPoint error = getRotatedError(theta, x, y);
		double targetHeading;
		this.straightPID.setFinishedRange(eps);
		this.turnPID.setMaxOutput(8);

		if (error.getY() < 0) { // flip X if we are going backwards
			error.setX(-error.getX());
		}

		double turningOffset = (error.getX() * turnRate); // based on how far we are in x turn more
		// System.out.println("turning offset" + turningOffset);
		if (turningOffset > maxTurn) {
			turningOffset = maxTurn;
		} else if (turningOffset < -maxTurn) {
			turningOffset = -maxTurn;
		}

		targetHeading = theta - turningOffset;
		this.turnPID.setDesiredValue(targetHeading);

		double yError = error.getY();
		double yOutput;

		/*
		 * if (Math.abs(yError) > 3.0) { this.robotOut.setDriveRampRate(0.20); } else {
		 * this.robotOut.setDriveRampRate(0); }
		 */

		yOutput = this.straightPID.calcPIDError(yError);

		double distanceFromTargetHeading = Math.abs(this.turnPID.getDesiredVal() - this.sensorIn.getAngle());
		if (distanceFromTargetHeading > 90) { // prevents the y output from being reversed in the next calculation
			distanceFromTargetHeading = 90;
		}

		yOutput = yOutput * (((-1 * distanceFromTargetHeading) / 90.0) + 1);

		//System.out.println("yOutput" + yOutput);
		//SmartDashboard.putNumber("YOUTPUT", yOutput);
		double xOutput = -this.turnPID.calcPID(this.sensorIn.getAngle());

		double leftOut = SimLib.calcLeftTankDrive(xOutput, yOutput);
		double rightOut = SimLib.calcRightTankDrive(xOutput, yOutput);

		this.setVelocityOutput(leftOut, rightOut);

		double dist = (yError);
		if (this.straightPID.isDone()) {
			//System.out.println("I have reached the epsilon!");
		}

		boolean isDone = false;
		if (minVelocity <= 0.5) {
			if (this.straightPID.isDone()) {
				disable();
				isDone = true;
				// this.robotOut.setDriveRampRate(0);
			}
		} else if (Math.abs(dist) < eps) {
			isDone = true;
			// this.robotOut.setDriveRampRate(0);
		}

		return isDone;
	}

	public void driveInAStrightLineTriangle() {
		if (sensorIn.getVisionTargetExists()) {
			double yError = sensorIn.getVisionDistanceFeet() - RobotConstants.TRIANGLE_SHOT_DISTANCE_FEET;
			double yOutput;
			yOutput = -this.straightPID.calcPIDError(yError);
			this.setVelocityOutput(yOutput, yOutput);
		} else {
			this.setVelocityOutput(0.0, 0.0);
		}
	}

	public void setState(DriveState state) {
		this.currentState = state;
	}

	public DriveState getState() {
		return this.currentState;
	}

	public SimPID getDriveTurnPID() {
		return this.turnPID;
	}

	public SimPID getDriveStraightPID() {
		return this.straightPID;
	}

	public SimPID getLimelightTurnPID() {
		return this.limelightTurnPID;
	}

	public LEDColourState getDesiredLedState() {
		return this.desiredLEDState;
	}

}