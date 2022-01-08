package frc.io;

import frc.util.LogitechF310Gamepad;

public class DriverInput {

	private static DriverInput instance;

	private LogitechF310Gamepad driver;
	private LogitechF310Gamepad operator;

	// Creates boolean variables that stores if a certain step/mode was pressed
	private boolean autonIncreaseStepWasPressed = false;
	private boolean autonDecreaseStepWasPressed = false;

	private boolean autonIncreaseModeWasPressed = false;
	private boolean autonDecreaseModeWasPressed = false;

	private boolean autonIncreaseMode10WasPressed = false;
	private boolean autonDecreaseMode10WasPressed = false;

	private DriverInput() {
		this.driver = new LogitechF310Gamepad(0);
		this.operator = new LogitechF310Gamepad(1);
	}

	public static DriverInput getInstance() {
		if (instance == null) {
			instance = new DriverInput();
		}
		return instance;
	}

	/*****************************
	 * DRIVER CONTROLS
	 *****************************/

	// DRIVE
	public double getDriverRightX() {
		return this.driver.getRightX();
	}

	public double getDriverLeftY() {
		return this.driver.getLeftY();
	}

	public double getDriverRightY(){
		return this.driver.getRightY();
	}

	public double getDriverLeftX(){
		return this.driver.getLeftX();
	}

	public boolean getDriverTeleopButton() {
		return this.driver.getGreenButton();
	}

	public boolean getDriverManualButton() {
		return this.driver.getStartButton();
	}

	public boolean getDriverManualntButton() {
		return this.driver.getBackButton();
	}

	public boolean getDriverHighGearButton(){
		return this.driver.getLeftBumper();
	}

	public boolean getDriverLowGearButton(){
		return Math.abs(this.driver.getLeftTrigger()) > 0.25;
	}

	public boolean getDriverDriveToCloseShotButton(){
		return this.driver.getYellowButton();
	}

	public double getOperatorRightX(){
		return this.operator.getRightX();
	}

	public double getOperatorRightY(){
		return this.operator.getRightY();
	}

	public double getOperatorLeftY(){
		return this.operator.getLeftY();
	}

	public double getOperatorLeftX(){
		return this.operator.getLeftX();
	}

	public boolean getOperatorArmsDownButton(){
		return this.operator.getGreenButton();
	}

	public boolean getOperatorRightLockButton(){
		return Math.abs(this.operator.getRightTrigger()) > 0.3;
	}

	public boolean getOperatorLeftLockButton(){
		return Math.abs(this.operator.getLeftTrigger()) > 0.3;
	}

	public boolean getOperatorRightUnlockButton(){
		return this.operator.getRightBumper();
	}

	public boolean getOperatorLeftUnlockButton(){
		return this.operator.getLeftBumper();
	}

	public boolean getOperatorTurretOffButton(){
		return this.operator.getRightBumper();
	}

	public boolean getOperatorTriangleButton(){
		return this.operator.getLeftBumper();
	}

	public boolean getDriverBaseLock(){
		return this.driver.getRightBumper();
	}

	public boolean getOperatorManualButton() {
		return this.operator.getStartButton();
	}

	public boolean getOperatorManualntButton() {
		return this.operator.getBackButton();
	}

	public boolean getOperatorStopCompressorButton() {
		return this.operator.getPOVDown();
	}

	public boolean getOperatorStartCompressorButton() {
		return this.operator.getPOVUp();
	}

	public boolean getOperatorIntakeOff(){//testing
		return operator.getRightBumper();
	}

	// SHOOTER
	public boolean getShootButton() {
		return this.driver.getRightTrigger() > 0.3;
	}

	public boolean getShooterOffButton() {
		return this.operator.getGreenButton();
	}

	public boolean getShooterOnButton(){
		return this.operator.getRedButton();
	}

	// TURRET
	public boolean getTurretRotateButton() {
		return this.driver.getGreenButton();
	}

	public boolean getTurretRotateToZeroButton() {
		return this.driver.getRedButton();
	}

	public boolean getAimTowardsGoalButton(){// not in use right now, maybe it won't be unless we can do the rezeroing thing
		return Math.abs(this.operator.getLeftTrigger()) > 0.3;
	}

	public boolean getStopIndexerIntakingButton(){
		return Math.abs(this.operator.getLeftTrigger()) > 0.3;
	}

	public boolean getTurnAimingOffButton(){
		return Math.abs(this.operator.getRightTrigger()) > 0.3;
	}

	// INTKAKE
	public boolean getIntakeForwardButton() {
		return this.driver.getGreenButton();
	}

	public boolean getIntakeReverseButton() {
		return this.driver.getRedButton();
	}

	public boolean getIntakeWristUpButton() {
		return this.driver.getBlueButton();
	}

	// INDEXER
	public boolean getIndexerMoveButton() {
		return this.driver.getBlueButton();
	}

	public boolean getHangerClimbButton(){
		return this.operator.getBlueButton();
	}

	public boolean getHangerArmsUpAndExtendButton(){
		return this.operator.getYellowButton();
	}

	public boolean getSpinIndexerUntilFiveBallsButton(){
		return this.operator.getRightBumper();
	}

	public boolean getSpinIndexer180Button(){
		return this.operator.getLeftBumper();
	}

	// COLOUR SPINNER
	public boolean getColourSpinnerPistonUpButton() {
		return this.operator.getPOVUp();
	}

	public boolean getColourSpinnerPistonDownButton() {
		return this.operator.getPOVDown();
	}

	public boolean getColourSpinnerRotationControlButton() {
		return this.operator.getPOVRight();
	}

	public boolean getColourSpinnerPositionControlButton() {
		return this.operator.getPOVLeft();
	}

	// ********************************
	// AUTO SELECTION CONTROLS
	// ********************************

	public boolean getDriverAutoOverrideButtons() {
		return this.driver.getGreenButton();
	}

	public boolean getOperatorAutoOverrideButtons() {
		return this.operator.getGreenButton();
	}

	public boolean getAutonSetDelayButton() {
		return false;// this.driver.getRightTrigger() > 0.2;
	}

	public double getAutonDelayStick() {
		return this.driver.getLeftY();
	}

	public boolean getAutonStepIncrease() {
		// only returns true on rising edge
		boolean result = this.driver.getRightBumper() && !this.autonIncreaseStepWasPressed;
		this.autonIncreaseStepWasPressed = this.driver.getRightBumper();
		return result;

	}

	public boolean getAutonStepDecrease() {
		// only returns true on rising edge
		boolean result = this.driver.getLeftBumper() && !this.autonDecreaseStepWasPressed;
		this.autonDecreaseStepWasPressed = this.driver.getLeftBumper();
		return result;

	}

	public boolean getAutonModeIncrease() {
		// only returns true on rising edge
		boolean result = this.driver.getRedButton() && !this.autonIncreaseModeWasPressed;
		this.autonIncreaseModeWasPressed = this.driver.getRedButton();
		return result;

	}

	public boolean getAutonModeDecrease() {
		// only returns true on rising edge
		boolean result = this.driver.getGreenButton() && !this.autonDecreaseModeWasPressed;
		this.autonDecreaseModeWasPressed = this.driver.getGreenButton();
		return result;

	}

	public boolean getAutonModeIncreaseBy10() {
		// only returns true on rising edge
		boolean result = this.driver.getYellowButton() && !this.autonIncreaseMode10WasPressed;
		this.autonIncreaseMode10WasPressed = this.driver.getYellowButton();
		return result;

	}

	public boolean getAutonModeDecreaseBy10() {
		// only returns true on rising edge
		boolean result = this.driver.getBlueButton() && !this.autonDecreaseMode10WasPressed;
		this.autonDecreaseMode10WasPressed = this.driver.getBlueButton();
		return result;

	}

}