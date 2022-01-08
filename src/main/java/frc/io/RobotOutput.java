package frc.io;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.util.PIDConstants;
import frc.util.SimEncoder;

public class RobotOutput {
	private static RobotOutput instance;

	private TalonFX driveL1;
	private TalonFX driveL2;
	private TalonFX driveR1;
	private TalonFX driveR2;

	private TalonFX rightHanger;
	private TalonFX leftHanger;

	private TalonSRX turret;
	private VictorSPX intake;

	private TalonSRX shooterMaster;
	private VictorSPX shooterSlave2;
	private VictorSPX shooterSlave1;
	private VictorSPX shooterSlave3;

	private PWM hood;

	private TalonFX indexerSpinner;
	private TalonSRX indexerElevator;

	private VictorSPX colourWheelSpinner;
	private Spark ledStrip;

	private Compressor compressor;
	private Solenoid colourWheelSpinnerUp;
	private Solenoid indexerWheelUp;
	private Solenoid indexerDoorClosed;
	private Solenoid climberArmsUp;
	private Solenoid climberArmsDown;
	private Solenoid climberLockOpen;
	private Solenoid intakeDown;
	private Solenoid lowGear;

	private boolean inLowGear = false;

	private double driveRampRate = 0;

	private double lastRightRampRate = 0;
	private double lastLeftRampRate = 0;

	private RobotOutput() {

		this.driveL1 = new TalonFX(3);
		this.driveL2 = new TalonFX(4);
		this.driveR1 = new TalonFX(1);
		this.driveR2 = new TalonFX(2);

		this.indexerSpinner = new TalonFX(5);
		this.indexerElevator = new TalonSRX(6);

		this.shooterMaster = new TalonSRX(12);//12 true good
		this.shooterSlave1 = new VictorSPX(8);//8
		this.shooterSlave2 = new VictorSPX(9);//9
		this.shooterSlave3 = new VictorSPX(10);//10

		this.hood = new PWM(9);//practice bot is 0

		this.turret = new TalonSRX(11);
		this.intake = new VictorSPX(7);
		this.colourWheelSpinner = new VictorSPX(13);
		this.rightHanger = new TalonFX(15);
		this.leftHanger = new TalonFX(14);
		System.out.println("creatint solenoids");

		this.ledStrip = new Spark(0); // PWM OUTPUT
		this.compressor = new Compressor();

		this.lowGear = new Solenoid(6);//prac 6
		this.climberArmsUp = new Solenoid(1);// prac 1
		this.climberArmsDown = new Solenoid(2);//prac 2
		this.climberLockOpen = new Solenoid(3);//prac 3
		this.intakeDown = new Solenoid(5);//prac 7
		this.indexerWheelUp = new Solenoid(7);//prac 4
		this.indexerDoorClosed = new Solenoid(4); // CANT BE 6,7,5,4,0 prac 5
		this.colourWheelSpinnerUp = new Solenoid(0);//prac 0

		this.configureSpeedControllers();

	}

	public static RobotOutput getInstance() {
		if (instance == null) {
			instance = new RobotOutput();
			System.out.println("GETTING INSTANCE OF ROBOTOUT");
		}
		return instance;
	}

	// Motor Commands

	public void configureSpeedControllers() {
		this.shooterMaster.configFactoryDefault();
		this.shooterSlave1.configFactoryDefault();
		this.shooterSlave2.configFactoryDefault();
		this.shooterSlave3.configFactoryDefault();
		
		this.shooterSlave3.follow(this.shooterMaster);
		this.shooterSlave2.follow(this.shooterMaster);
		this.shooterSlave1.follow(this.shooterMaster);

		this.shooterMaster.setInverted(true);
		this.shooterSlave1.setInverted(true);
		this.shooterSlave2.setInverted(false);
		this.shooterSlave3.setInverted(false);
		

		this.shooterMaster.setNeutralMode(NeutralMode.Coast);
		this.shooterSlave1.setNeutralMode(NeutralMode.Coast);
		this.shooterSlave2.setNeutralMode(NeutralMode.Coast);
		this.shooterSlave3.setNeutralMode(NeutralMode.Coast);

		this.shooterMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
		this.shooterMaster.configVelocityMeasurementWindow(1);

		this.driveL2.follow(this.driveL1);
		this.driveR2.follow(this.driveR1);

		this.driveL1.setInverted(false);
		this.driveL2.setInverted(false);
		this.driveR1.setInverted(true);
		this.driveR2.setInverted(true);

		this.driveL1.setNeutralMode(NeutralMode.Coast);
		this.driveL2.setNeutralMode(NeutralMode.Coast);
		this.driveR1.setNeutralMode(NeutralMode.Coast);
		this.driveR2.setNeutralMode(NeutralMode.Coast);

		this.indexerSpinner.setNeutralMode(NeutralMode.Coast);
		this.indexerSpinner.configOpenloopRamp(0.5);
		this.indexerSpinner.configClosedloopRamp(0.5);
		this.indexerSpinner.setInverted(false);
		this.indexerElevator.setInverted(true);
		this.indexerElevator.setNeutralMode(NeutralMode.Coast);
		this.indexerElevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		this.indexerElevator.setSensorPhase(true);
		this.intake.setNeutralMode(NeutralMode.Coast);
		this.intake.setInverted(false);
		this.turret.setInverted(true);
		this.turret.setNeutralMode(NeutralMode.Brake);

		this.shooterMaster.setSensorPhase(true);
		this.shooterMaster.configClosedloopRamp(0.2);

		this.leftHanger.setNeutralMode(NeutralMode.Brake);
		this.rightHanger.setNeutralMode(NeutralMode.Brake);

		this.leftHanger.setInverted(false);
		this.rightHanger.setInverted(true);
		SupplyCurrentLimitConfiguration hangerCurrent = new SupplyCurrentLimitConfiguration(true,80,120,0.5);
		this.leftHanger.configSupplyCurrentLimit(hangerCurrent);
		this.rightHanger.configSupplyCurrentLimit(hangerCurrent);

		this.hood.setBounds(2.5, 1.52, 1.5, 1.48, 0.5);
	
	}

	// Shooter

	public void setTurretOutput(double output) {
		this.turret.set(ControlMode.PercentOutput, output);
	}

	public void setTurretSpeed(double angle) {
		this.turret.set(ControlMode.Velocity, angle);
	}

	public void setShooterOutput(double output) {
		this.shooterMaster.set(ControlMode.PercentOutput, output);
	}

	public void setShooterVelocity(double velocity){
		this.shooterMaster.set(ControlMode.Velocity, velocity);
	}

	public void setShooterRPM(double rpm){
		setShooterVelocity((rpm * RobotConstants.SHOOTER_TICKS_PER_REV / 60) / 10);
	}

	public void resetShooterMotors(){
		this.shooterMaster.configFactoryDefault();
		this.shooterSlave1.configFactoryDefault();
		this.shooterSlave2.configFactoryDefault();
		this.shooterSlave3.configFactoryDefault();

		this.shooterMaster.setInverted(true);
		this.shooterSlave1.setInverted(true);
		this.shooterSlave2.setInverted(false);
		this.shooterSlave3.setInverted(false);

		this.shooterMaster.setNeutralMode(NeutralMode.Coast);
		this.shooterSlave1.setNeutralMode(NeutralMode.Coast);
		this.shooterSlave2.setNeutralMode(NeutralMode.Coast);
		this.shooterSlave3.setNeutralMode(NeutralMode.Coast);
	}

	public void setShooterSlave1Output(double output){
		this.shooterSlave1.set(ControlMode.PercentOutput, output);
	}

	public void setShooterSlave2Output(double output){
		this.shooterSlave2.set(ControlMode.PercentOutput, output);
	}

	public void setShooterSlave3Output(double output){
		this.shooterSlave3.set(ControlMode.PercentOutput, output);
	}

	public void setHood(double speed){
		if(speed > 0.99) speed = 0.99;
		if(speed < -0.99) speed = -.99;
		this.hood.setSpeed(-speed);
	}

	public double getShooterMasterEncoderRaw(){
		return this.shooterMaster.getSelectedSensorPosition();
	}

	public double getShooterMasterTicksPer100ms(){
		return this.shooterMaster.getSelectedSensorVelocity();
	}

	public double getShooterOutput(){
		return this.shooterMaster.getMotorOutputPercent();
	}

	public double getTurretEncoderPosition(){
		return this.turret.getSelectedSensorPosition();
	}

	public double getTurretTicksPer100ms(){
		return this.turret.getSelectedSensorVelocity();
	}

	public void resetTurretEncoder(){
		this.turret.setSelectedSensorPosition(0);
	}

	public void setTurretPositionDegrees(double degrees){
		if(degrees > RobotConstants.TURRET_MAX_ANGLE){
			degrees = RobotConstants.TURRET_MAX_ANGLE;
		} else if(degrees < RobotConstants.TURRET_MIN_ANGLE){
			degrees = RobotConstants.TURRET_MIN_ANGLE;
		}
		this.turret.set(ControlMode.Position, degrees * RobotConstants.TURRET_TICKS_PER_DEGREE);
	}

	public void configureTurretPID(PIDConstants constants){
		this.turret.config_kP(0, constants.p);
		this.turret.config_kI(0, constants.i);
		this.turret.config_kD(0, constants.d);
		this.turret.config_IntegralZone(0, 5000);
		this.turret.configAllowableClosedloopError(0, (int) constants.eps);
	}

	// Drive

	public void setDriveLeft(double output) {
		this.driveL1.set(ControlMode.PercentOutput, output);
	}

	public void setDriveRight(double output) {
		this.driveR1.set(ControlMode.PercentOutput, output);
	}

	public void setDriveLeftVelocity(double velocity) {
		this.driveL1.set(ControlMode.Velocity, velocity);
		
	}

	public void setDriveRightVelocity(double velocity) {
		this.driveR1.set(ControlMode.Velocity, velocity);
		
	}

	public void setDriveLeftFPS(double fps) {
		if (this.inLowGear) {
			setDriveLeftVelocity(fps * RobotConstants.DRIVE_TICKS_PER_FOOT_LOW / 10.0);
		} else {
			setDriveLeftVelocity(fps * RobotConstants.DRIVE_TICKS_PER_FOOT_HIGH / 10.0);
		}
	}

	public void setDriveRightFPS(double fps) {
		if (this.inLowGear) {
			setDriveRightVelocity(fps * RobotConstants.DRIVE_TICKS_PER_FOOT_LOW / 10.0);
		} else {
			setDriveRightVelocity(fps * RobotConstants.DRIVE_TICKS_PER_FOOT_HIGH / 10.0);
		}
	}

	public double getLeftDriveFPS() {
		if (inLowGear) {
			return this.driveL1.getSelectedSensorVelocity() * 10.0 / RobotConstants.DRIVE_TICKS_PER_FOOT_LOW;
		} else {
			return this.driveL1.getSelectedSensorVelocity() * 10.0 / RobotConstants.DRIVE_TICKS_PER_FOOT_HIGH;
		}
	}

	public double getRightDriveFPS() {
		if (inLowGear) {
			return this.driveR1.getSelectedSensorVelocity() * 10.0 / RobotConstants.DRIVE_TICKS_PER_FOOT_LOW;
		} else {
			
			return this.driveR1.getSelectedSensorVelocity() * 10.0 / RobotConstants.DRIVE_TICKS_PER_FOOT_HIGH;
			
		}
	}

	public boolean isInLowGear(){
		return this.inLowGear;
	}

	public double getDriveFPSAverage(){
		return (getRightDriveFPS() + getLeftDriveFPS()) / 2.0;
	}

	public double getLeftDriveAveragePosition(){
		return (driveL1.getSelectedSensorPosition() + driveL2.getSelectedSensorPosition()) / 2;
	}

	public double getRightDriveAveragePosition(){
		return (driveR1.getSelectedSensorPosition() + driveR2.getSelectedSensorPosition()) / 2;
	}

	public double getDriveAveragePosition(){
		return (getRightDriveAveragePosition() + getLeftDriveAveragePosition()) / 2;
	}

	public void resetDriveEncoders(){
		this.driveL1.setSelectedSensorPosition(0);
		this.driveL2.setSelectedSensorPosition(0);
		this.driveR1.setSelectedSensorPosition(0);
		this.driveR2.setSelectedSensorPosition(0);
	}

	public void setDriveRampRate(double rampRateSecondsToFull) {
		this.driveL1.configOpenloopRamp(rampRateSecondsToFull);
		this.driveL2.configOpenloopRamp(rampRateSecondsToFull);
		this.driveR1.configOpenloopRamp(rampRateSecondsToFull);
		this.driveR2.configOpenloopRamp(rampRateSecondsToFull);
		this.driveRampRate = rampRateSecondsToFull;
	}

	public void setDriveClosedLoopRampRate(double rampRateSecondsToFull) {
		this.driveL1.configClosedloopRamp(rampRateSecondsToFull);
		this.driveL2.configClosedloopRamp(rampRateSecondsToFull);
		this.driveR1.configClosedloopRamp(rampRateSecondsToFull);
		this.driveR2.configClosedloopRamp(rampRateSecondsToFull);
		this.driveRampRate = rampRateSecondsToFull;
	}

	public void configureShooterPID(PIDConstants constants){
		this.shooterMaster.config_kP(0, constants.p);
		this.shooterMaster.config_kI(0, constants.i);
		this.shooterMaster.config_kD(0, constants.d);
		this.shooterMaster.config_IntegralZone(0, 3000);
		this.shooterMaster.configAllowableClosedloopError(0, (int) constants.eps);
		this.shooterMaster.config_kF(0, constants.ff);
	}

	public void configureHighGearVelPID(PIDConstants constants) {
		this.driveL1.config_kP(0, constants.p);
		this.driveR1.config_kP(0, constants.p);
	
		this.driveL1.config_kI(0, constants.i);
		this.driveR1.config_kI(0, constants.i);
	

		this.driveL1.config_kD(0, constants.d);
		this.driveR1.config_kD(0, constants.d);
	

		this.driveL1.configAllowableClosedloopError(0, (int) constants.eps);
		this.driveR1.configAllowableClosedloopError(0, (int) constants.eps);
		
		this.driveL1.config_IntegralZone(0, 10000);
		this.driveR1.config_IntegralZone(0, 10000);
		
		this.driveL1.config_kF(0, constants.ff);
		this.driveR1.config_kF(0, constants.ff);
	}

	public void configureLowGearVelPID(PIDConstants constants) {
		this.driveL1.config_kP(1, constants.p);
		this.driveL2.config_kP(1, constants.p);
		this.driveR1.config_kP(1, constants.p);
		this.driveR2.config_kP(1, constants.p);

		this.driveL1.config_kI(1, constants.i);
		this.driveL2.config_kI(1, constants.i);
		this.driveR1.config_kI(1, constants.i);
		this.driveR2.config_kI(1, constants.i);

		this.driveL1.config_kD(1, constants.d);
		this.driveL2.config_kD(1, constants.d);
		this.driveR1.config_kD(1, constants.d);
		this.driveR2.config_kD(1, constants.d);

		this.driveL1.configAllowableClosedloopError(1, (int) constants.eps);
		this.driveL2.configAllowableClosedloopError(1, (int) constants.eps);
		this.driveR1.configAllowableClosedloopError(1, (int) constants.eps);
		this.driveR2.configAllowableClosedloopError(1, (int) constants.eps);

		this.driveL1.config_IntegralZone(1, 10000);
		this.driveL2.config_IntegralZone(1, 10000);
		this.driveR1.config_IntegralZone(1, 10000);
		this.driveR2.config_IntegralZone(1, 10000);

		this.driveL1.config_kF(1, constants.ff);
		this.driveL2.config_kF(1, constants.ff);
		this.driveR1.config_kF(1, constants.ff);
		this.driveR2.config_kF(1, constants.ff);
	}

	public void setLowGear(boolean lowGear) {
		this.lowGear.set(lowGear);
		this.inLowGear = lowGear;
	}

	public double getDriveRampRate() {
		return this.driveRampRate;
	}

	public double getDriveL1Encoder() {
		return driveL1.getSelectedSensorPosition();
	}

	public double getDriveL2Encoder() {
		return driveL2.getSelectedSensorPosition();
	}

	public double getDriveR1Encoder() {
		return driveR1.getSelectedSensorPosition();
	}

	public double getDriveR2Encoder() {
		return driveR2.getSelectedSensorPosition();
	}

	public double getDriveL1Velocity() {
		return driveL1.getSelectedSensorVelocity();
	}

	public double getDriveL2Velocity() {
		return driveL2.getSelectedSensorVelocity();
	}

	public double getDriveR1Velocity() {
		return driveR1.getSelectedSensorVelocity();
	}

	public double getDriveR2Velocity() {
		return driveR2.getSelectedSensorVelocity();
	}

	// Intake

	public void setIntakeOutput(double output) {
		this.intake.set(ControlMode.PercentOutput, output);
	}

	public void setIntakeWrist(boolean up) {
		this.intakeDown.set(up);
	}

	// Indexer

	public void setIndexerSpinner(double output) {
		this.indexerSpinner.set(ControlMode.PercentOutput, output);
	}

	public void setIndexerElevator(double output) {
		this.indexerElevator.set(ControlMode.PercentOutput, output);
	}

	public void setIndexerSpinnerPosition(double rawPosition) {
		this.indexerSpinner.set(ControlMode.Position, rawPosition);
	}

	public void setIndexerSpinnerRPM(double RPM) {
		this.indexerSpinner.set(ControlMode.Velocity, RPM * RobotConstants.INDEXER_TICKS_PER_REV / 600.0);
	}

	public void setIndexerElevatorRPM(double RPM) {
		this.indexerElevator.set(ControlMode.Velocity, RPM * RobotConstants.INDEXER_ELEVATOR_TICKS_PER_REV / 600.0);
	}

	public void setIndexerElevatorRawVelocity(double velocity) {
		this.indexerElevator.set(ControlMode.Velocity, velocity);
	}

	public double getRawIndexerSpinnerPosition() {
		return this.indexerSpinner.getSelectedSensorPosition();
	}

	public double getRawIndexerSpinnerVelocity() {
		return this.indexerSpinner.getSelectedSensorVelocity();
	}

	public double getRawIndexerElevatorVelocity() {
		return this.indexerElevator.getSelectedSensorVelocity();
	}

	public double getRawIndexerElevatorPosition() {
		return this.indexerElevator.getSelectedSensorPosition();
	}

	public void setIndexerSpinnerNeutralMode(NeutralMode mode){
		this.indexerSpinner.setNeutralMode(mode);
	}

	public void resetIndexerEncoder() {
		this.indexerSpinner.setSelectedSensorPosition(0 - (int) (25 * (RobotConstants.INDEXER_TICKS_PER_REV / 360))); // we
																														// want
																														// to
																														// zero
																														// with
																														// 0
																														// being
																														// straight
																														// up
	}

	public void configureIndexerSpinnerPID(PIDConstants constants) {
		this.indexerSpinner.config_kP(0, constants.p);
		this.indexerSpinner.config_kI(0, constants.i);
		this.indexerSpinner.config_kD(0, constants.d);
		this.indexerSpinner.configAllowableClosedloopError(0, (int) constants.eps);
		this.indexerSpinner.config_IntegralZone(0, 10000);
		this.indexerSpinner.config_kF(0, constants.ff);
	}

	public void configureIndexerElevatorPID(PIDConstants constants) {
		this.indexerElevator.config_kP(0, constants.p);
		this.indexerElevator.config_kI(0, constants.i);
		this.indexerElevator.config_kD(0, constants.d);
		this.indexerElevator.configAllowableClosedloopError(0, (int) constants.eps);
		this.indexerElevator.config_IntegralZone(0, 10000);
		this.indexerElevator.config_kF(0, constants.ff);
	}

	// Colour Wheel Spinner

	public void setColourWheelOutput(double output) {
		this.colourWheelSpinner.set(ControlMode.PercentOutput, output);
	}

	public void setColourWheelUp(boolean up) {
		this.colourWheelSpinnerUp.set(up);
	}

	// Hanger

	public void configureHangerUpPID(PIDConstants constants) {
		this.rightHanger.config_kP(0, constants.p);
		this.rightHanger.config_kI(0, constants.i);
		this.rightHanger.config_kD(0, constants.d);
		this.rightHanger.configAllowableClosedloopError(0, (int) constants.eps);
		this.rightHanger.config_IntegralZone(0, 10000);
	

		this.leftHanger.config_kP(0, constants.p);
		this.leftHanger.config_kI(0, constants.i);
		this.leftHanger.config_kD(0, constants.d);
		this.leftHanger.configAllowableClosedloopError(0, (int) constants.eps);
		this.leftHanger.config_IntegralZone(0, 10000);
		
	}

	public void configureHangerDownPID(PIDConstants constants) {
		this.rightHanger.config_kP(1, constants.p);
		this.rightHanger.config_kI(1, constants.i);
		this.rightHanger.config_kD(1, constants.d);
		this.rightHanger.configAllowableClosedloopError(1, (int) constants.eps);
		this.rightHanger.config_IntegralZone(1, 10000);
	

		this.leftHanger.config_kP(1, constants.p);
		this.leftHanger.config_kI(1, constants.i);
		this.leftHanger.config_kD(1, constants.d);
		this.leftHanger.configAllowableClosedloopError(1, (int) constants.eps);
		this.leftHanger.config_IntegralZone(1, 10000);
		
	}

	public void setHangerPIDSlot(int slot, int pidIdx){
		this.leftHanger.selectProfileSlot(slot, pidIdx);
		this.rightHanger.selectProfileSlot(slot, pidIdx);
	}

	public void setHangerPIDSlot(int slot){
		this.leftHanger.selectProfileSlot(slot, 0);
		this.rightHanger.selectProfileSlot(slot, 0);
	}

	public void setRightHangerOutput(double output) {
		this.rightHanger.set(ControlMode.PercentOutput, output);
	}

	public void setRightHangerPosition(double position) {
		this.rightHanger.set(ControlMode.Position, position);
	}

	public void setRightHangerVelocity(double velocity) {
		this.rightHanger.set(ControlMode.Velocity, velocity);
	}

	public void setLeftHangerOutput(double output) {
		this.leftHanger.set(ControlMode.PercentOutput, output);
	}

	public void setLeftHangerPosition(double position) {
		this.leftHanger.set(ControlMode.Position, position);
	}

	public void setLeftHangerVelocity(double velocity) {
		this.leftHanger.set(ControlMode.Velocity, velocity);
	}

	public void setHangerOutput(double output) {
		setRightHangerOutput(output);
		setLeftHangerOutput(output);
	}

	public void setHangerPosition(double position) {
		setRightHangerPosition(position);
		setLeftHangerPosition(position);
	}

	public double getLeftHangerPosition(){
		return this.leftHanger.getSelectedSensorPosition();
	}

	public double getRightHangerPosition(){
		return this.rightHanger.getSelectedSensorPosition();
	}

	public void resetClimbEncoders(){
		this.rightHanger.setSelectedSensorPosition(0);
		this.leftHanger.setSelectedSensorPosition(0);
	}

	public double getRightHangerPercentOutput(){
		return this.rightHanger.getMotorOutputPercent();
	}

	public double getLeftHangerPercentOutput(){
		return this.leftHanger.getMotorOutputPercent();
	}

	public void setLeftHangerRampRate(double rampRate){
		if(this.lastLeftRampRate != rampRate){
			this.leftHanger.configOpenloopRamp(rampRate);
			this.lastLeftRampRate = rampRate;
		}
	}

	public void setRightHangerRampRate(double rampRate){
		if(this.lastRightRampRate != rampRate){
			this.rightHanger.configOpenloopRamp(rampRate);
			this.lastRightRampRate = rampRate;
		}
	}

	public void setClimberArms(boolean up) {
		this.climberArmsUp.set(up);
		this.climberArmsDown.set(!up);
	}

	public void setClimberLock(boolean locked) {
		this.climberLockOpen.set(!locked);
	}

	public void setCompressor(boolean on) {
		if (on) {
			this.compressor.start();
		} else {
			this.compressor.stop();
		}
	}

	public boolean getCompressorState() {
		return this.compressor.enabled();
	}

	public void setLEDStrip(double pattern) {
		ledStrip.set(pattern);
	}

	public double getLEDStrip() {
		return this.ledStrip.get();
	}

	public void setIndexerWheel(boolean on) {
		this.indexerWheelUp.set(on);
	}

	public void setIndexerDoorClosed(boolean closed) {
		this.indexerDoorClosed.set(closed);
	}

	public void stopAll() {
		setDriveLeft(0);
		setDriveRight(0);
		setShooterOutput(0);
		setIntakeOutput(0.0);
		// shut off things here
	}

}
