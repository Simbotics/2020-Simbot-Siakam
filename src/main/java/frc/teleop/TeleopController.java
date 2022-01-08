package frc.teleop;

import frc.io.DriverInput;
import frc.io.RobotOutput;
import frc.io.SensorInput;
import frc.robot.RobotConstants;
import frc.subsystems.Drive;
import frc.subsystems.Indexer;
import frc.subsystems.Intake;
import frc.subsystems.Drive.DriveState;
import frc.subsystems.Hanger.HangerState;
import frc.subsystems.Indexer.IndexerState;
import frc.subsystems.Intake.IntakeState;
import frc.subsystems.LEDStrip;
import frc.subsystems.Shooter;
import frc.subsystems.Turret;
import frc.subsystems.Hanger;
import frc.subsystems.ColourSpinner;
import frc.subsystems.LEDStrip.LEDColourState;
import frc.subsystems.Shooter.ShooterState;
import frc.subsystems.Turret.TurretState;
import frc.util.SimLib;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleopController extends TeleopComponent {

	private static TeleopController instance;
	private DriverInput driverIn;
	private SensorInput sensorIn;
	private RobotOutput robotOut;
	private Drive drive;
	private Shooter shooter;
	private Intake intake;
	private Turret turret;
	private Indexer indexer;
	private Hanger hanger;
	private LEDStrip led;
	private boolean driverManual = false;
	private boolean operatorManual = false;
	private LEDColourState desiredLedState = LEDColourState.OFF;

	private boolean hasArmsUpBeenPressed = false;

	public static TeleopController getInstance() {
		if (instance == null) {
			instance = new TeleopController();
		}
		return instance;
	}

	private TeleopController() {
		this.drive = Drive.getInstance();
		// this.led = LEDStrip.getInstance();
		this.sensorIn = SensorInput.getInstance();
		this.driverIn = DriverInput.getInstance();
		this.robotOut = RobotOutput.getInstance();
		this.shooter = Shooter.getInstance();
		this.intake = Intake.getInstance();
		this.turret = Turret.getInstance();
		this.indexer = Indexer.getInstance();
		this.hanger = Hanger.getInstance();

	}

	@Override
	public void firstCycle() {
		this.drive.firstCycle();
		this.shooter.firstCycle();
		this.intake.firstCycle();
		this.indexer.firstCycle();
		this.turret.firstCycle();
		this.hanger.firstCycle();

	}

	@Override
	public void calculate() {

		////////////////////////////////////
		///////// MANUAL TOGGLE ////////////
		////////////////////////////////////

		if (this.driverIn.getDriverManualButton()) {
			this.driverManual = true;
		} else if (this.driverIn.getDriverManualntButton()) {
			this.driverManual = false;
		}

		if (this.driverIn.getOperatorManualButton()) {
			this.operatorManual = true;
		} else if (this.driverIn.getOperatorManualntButton()) {
			this.operatorManual = false;
		}

		SmartDashboard.putBoolean("operator manual?", this.operatorManual);
		SmartDashboard.putBoolean("driver manual?", this.driverManual);

		//////////////////////////////
		///////// DRIVE CODE /////////
		//////////////////////////////

		// ΜΑNUAL
		if (driverManual) {
			double x = this.driverIn.getDriverRightX();
			double y = this.driverIn.getDriverLeftY();

			if (Math.abs(x) < 0.05) {
				x = 0;
			}
			if (Math.abs(y) < 0.1) {
				y = 0;
			}

			x = SimLib.squareMaintainSign(x);
			y = SimLib.squareMaintainSign(y);

			this.drive.setOutput(y, x);

			this.drive.setState(DriveState.OUTPUT);

			if (this.driverIn.getDriverHighGearButton()) {
				this.robotOut.setLowGear(false);
			} else if (this.driverIn.getDriverLowGearButton()) {
				this.robotOut.setLowGear(true);
			}

		} else { // Driver non-manual

			double x = this.driverIn.getDriverRightX();
			double y = this.driverIn.getDriverLeftY();

			if (Math.abs(x) < 0.05) {
				x = 0;
			}
			if (Math.abs(y) < 0.1) {
				y = 0;
			}

			x = SimLib.squareMaintainSign(x);
			y = SimLib.squareMaintainSign(y);

			this.drive.setOutput(y, x);

			this.drive.setState(DriveState.OUTPUT);

			//////////////////////////////
			///////// Drive CODE ////////
			//////////////////////////////

			if (this.driverIn.getDriverHighGearButton()) {
				this.robotOut.setLowGear(false);
			} else if (this.driverIn.getDriverLowGearButton()) {
				this.robotOut.setLowGear(true);
			}

			if (this.driverIn.getDriverBaseLock() && this.shooter.getState() == ShooterState.TRIANGLE_SHOT) {
				this.drive.setState(DriveState.DRIVING_TO_TRIANGLE);
			}

			// DRIVER SHOOTING BUTTONS

			/*
			 * if (this.driverIn.getShootButton() && this.shooter.isShooterAtSpeedAndAimed()
			 * && this.shooter.isTurretAimedAtVisionTarget()) {
			 * this.indexer.setState(IndexerState.SHOOTING); } else if
			 * (this.indexer.getState() != IndexerState.BRAKING_WITH_BALL_IN_PLACE &&
			 * this.indexer.getState() != IndexerState.STICK_CONTROL &&
			 * this.shooter.getState() != ShooterState.GETTING_TO_SPEED &&
			 * this.shooter.getState() != ShooterState.SHOOTING && this.indexer.getState()
			 * != IndexerState.INTAKING) {
			 * this.indexer.setState(IndexerState.WAITING_FOR_SHOOTER); } else if
			 * (!this.driverIn.getShootButton() && this.shooter.getState() ==
			 * ShooterState.SHOOTING) {
			 * this.shooter.setState(ShooterState.GETTING_TO_SPEED); }
			 */

			//////////////////////////////
			///////// INTAKE CODE ////////
			//////////////////////////////

			if (this.driverIn.getIntakeForwardButton()) {
				if (!this.driverIn.getShootButton()) {
					this.indexer.setState(IndexerState.INTAKING);
				}
				if (this.intake.getState() == IntakeState.OFF_DOWN
						|| this.intake.getState() == IntakeState.CLIMBING_START) {
					this.intake.setState(IntakeState.INTAKING_DOWN);
				} else if (this.intake.getState() == IntakeState.INTAKING_UP
						|| this.intake.getState() == IntakeState.OFF_UP
						|| this.intake.getState() == IntakeState.RETRACTING_IN) {
					this.intake.setState(IntakeState.EXTENDING_DOWN);
				}
			} else if (this.driverIn.getIntakeReverseButton()) {
				this.intake.setState(IntakeState.REVERSE);
			} else if (this.driverIn.getIntakeWristUpButton()) {
				this.intake.setState(IntakeState.RETRACTING_IN);
			} else {
				if(this.intake.getState() == IntakeState.REVERSE){
					this.intake.setState(IntakeState.OFF_DOWN);
				}
				if (this.intake.getState() == IntakeState.INTAKING_DOWN
						|| this.intake.getState() == IntakeState.EXTENDING_DOWN) {
					this.intake.setState(IntakeState.OFF_DOWN);
				}
				if (this.intake.getState() == IntakeState.INTAKING_UP
						|| this.intake.getState() == IntakeState.RETRACTING_IN) {
					this.intake.setState(IntakeState.OFF_UP);
				}
			}

			/*
			 * if (this.driverIn.getIntakeForwardButton() || this.indexer.getState() ==
			 * IndexerState.INTAKING) { if (this.intake.getState() !=
			 * IntakeState.INTAKING_DOWN || this.intake.getState() !=
			 * IntakeState.EXTENDING_DOWN) {
			 * this.intake.setState(IntakeState.INTAKING_DOWN); }
			 * if(!this.driverIn.getIntakeForwardButton()){
			 * this.intake.setState(IntakeState.OFF_); } if (this.indexer.getState() !=
			 * IndexerState.STICK_CONTROL) {
			 * System.out.println("I SET THE INDEXER TO INTAKE MODE");
			 * this.indexer.setState(IndexerState.INTAKING); } } else if
			 * (this.driverIn.getIntakeReverseButton()) {
			 * this.intake.setState(IntakeState.REVERSE); } else if
			 * (this.driverIn.getIntakeWristUpButton()) {
			 * this.intake.setState(IntakeState.RETRACTING_IN); } else {
			 * this.intake.setState(IntakeState.OFF); }
			 */
		}

		if (this.operatorManual) {

			this.hanger.setState(HangerState.MANUAL);

		} else { // Operator non-manual

			////////////////////////////
			//////// SHOOTER CODE //////
			////////////////////////////

			if (this.driverIn.getShooterOnButton()) {
				this.shooter.setState(ShooterState.SHOOTING);
			} else if (this.driverIn.getShooterOffButton()) {
				this.shooter.setState(ShooterState.OFF);
			}

			if (this.driverIn.getShootButton()) {
				if (this.indexer.getState() != IndexerState.BRAKING_WITH_BALL_IN_PLACE
						&& this.indexer.getState() != IndexerState.SHOOTING) {
					this.indexer.setState(IndexerState.LINING_UP_SLOT);
				}
				if (this.indexer.getState() == IndexerState.BRAKING_WITH_BALL_IN_PLACE) {
					if (this.shooter.isTurretAimedAtVisionTarget() && this.shooter.isShooterAtSpeedAndAimed()) {
						this.indexer.setState(IndexerState.SHOOTING);
					}
				}
			} else if (this.indexer.getState() == IndexerState.SHOOTING) {
				this.indexer.setState(IndexerState.POST_SHOOTING);
			}

			// OPERATOR INDEXER STATE TRANSITIONS

			if (this.indexer.getState() != IndexerState.SHOOTING && this.indexer.getState() != IndexerState.POST_SHOOTING 
				&& this.indexer.getState() != IndexerState.UNJAM) {
				if (this.indexer.getState() == IndexerState.INTAKING) {
					if (this.driverIn.getStopIndexerIntakingButton()) {
						this.indexer.setState(IndexerState.LINING_UP_SLOT);
					}
				} else {
					if (this.indexer.getState() != IndexerState.BRAKING_WITH_BALL_IN_PLACE) {
						if (this.indexer.getLastCycleState() == IndexerState.LINING_UP_SLOT) {
							if (this.indexer.isIndexerSpinnerPIDDone()) {
								this.indexer.setState(IndexerState.BRAKING_WITH_BALL_IN_PLACE);
							}
						} else {
							this.indexer.setState(IndexerState.LINING_UP_SLOT);
						}
					}
				}
			}

			if(this.driverIn.getColourSpinnerPistonDownButton()){
				this.indexer.setState(IndexerState.UNJAM);
			}else if(!this.driverIn.getColourSpinnerPistonDownButton() && this.indexer.getState() == IndexerState.UNJAM){
				this.indexer.setState(IndexerState.LINING_UP_SLOT);
			}

			/*if (this.indexer.getState() != IndexerState.SHOOTING && this.shooter.getState() != ShooterState.SHOOTING
					&& this.shooter.getState() != ShooterState.GETTING_TO_SPEED) {
				if (this.indexer.getState() != IndexerState.INTAKING) {
					if (this.indexer.getState() != IndexerState.BRAKING_WITH_BALL_IN_PLACE) {
						if (this.indexer.isIndexerSpinnerPIDDone()
								&& this.indexer.getLastCycleState() == IndexerState.WAITING_FOR_SHOOTER) {
							this.indexer.setState(IndexerState.BRAKING_WITH_BALL_IN_PLACE);
						} else {
							this.indexer.setState(IndexerState.WAITING_FOR_SHOOTER);
						}
					}
				} else if (this.indexer.getState() == IndexerState.INTAKING
						&& this.driverIn.getStopIndexerIntakingButton()) {
					this.indexer.setState(IndexerState.WAITING_FOR_SHOOTER);
				}
			}*/

			//////////////////////////////
			///////// TURRET CODE ////////
			//////////////////////////////

			if (this.driverIn.getOperatorTurretOffButton()) {
				this.turret.setTurretTargetAngle(0);
				this.turret.setState(TurretState.ROTATING_TO_ANGLE);
				this.shooter.setState(ShooterState.UNDER_TRENCH);
			} else if (this.driverIn.getOperatorTriangleButton()) {
				this.shooter.setState(ShooterState.TRIANGLE_SHOT);
				this.turret.setTurretTargetAngle(180);
				this.turret.setState(TurretState.OPERATOR_POINTING);
			}

			if (this.indexer.getState() == IndexerState.INTAKING 
					&& this.shooter.getState() != ShooterState.SHOOTING
					&& this.turret.getState() != TurretState.OPERATOR_POINTING
					&& this.turret.getState() != TurretState.VISION_CONTROL
					&& this.hanger.getState() == HangerState.NOT_HANGING) {
				this.turret.setTurretTargetAngle(180);
				this.turret.setState(TurretState.ROTATING_TO_ANGLE);
			}

			if (this.turret.getState() != TurretState.VISION_CONTROL && this.hanger.getState() == HangerState.NOT_HANGING) {

				double stickDistance = Math.sqrt(
						Math.pow(this.driverIn.getOperatorLeftY(), 2) + Math.pow(this.driverIn.getOperatorLeftX(), 2));

				//SmartDashboard.putNumber("STICK DISTANCE", stickDistance);

				if (stickDistance > RobotConstants.TURRET_DEADZONE_EPS) {

					double stickAngle = Math
							.toDegrees(Math.atan2(-this.driverIn.getOperatorLeftX(), this.driverIn.getOperatorLeftY()));
					//SmartDashboard.putNumber("STICK ANGLE", stickAngle);

					double angleFromGyroForTurret = this.sensorIn.getGyroAngle();

					if (angleFromGyroForTurret < 0) {
						angleFromGyroForTurret = 360 + (angleFromGyroForTurret % 360);
					} else {
						angleFromGyroForTurret %= 360;
					}

					//SmartDashboard.putNumber("ANGLE FROM GYRO CONVERTED TO POSITIVE", angleFromGyroForTurret);

					double angle = (stickAngle - angleFromGyroForTurret) + 90;
					//SmartDashboard.putNumber("TURRET ANGLE FROM THE STICK", angle);

					if (angle > 360) {
						angle %= 360;
					} else if (angle < -90) {
						angle += 360;
					}

					//SmartDashboard.putNumber("TURRET ANGLE FROM THE STICK ADJUSTED IN CASE ITS < -90", angle);

					this.turret.setTurretTargetAngle(angle);

					this.turret.setState(TurretState.OPERATOR_POINTING);
				}
			}

			// kalebs climbing code
			if (this.driverIn.getHangerArmsUpAndExtendButton() && this.driverIn.getColourSpinnerPistonUpButton()) { // we want to get ready to climb now
				if (this.hanger.getState() == HangerState.NOT_HANGING) {
					if (this.intake.getState() == IntakeState.OFF_DOWN
							|| this.intake.getState() == IntakeState.CLIMBING_START) { // intake down so all good
						this.hanger.setState(HangerState.WAITING_FOR_CYLINDERS);
						this.intake.setState(IntakeState.CLIMBING_START);
					} else {
						this.intake.setState(IntakeState.EXTENDING_DOWN_CLIMBING);
					}
				}
			}

			if (this.intake.getState() == IntakeState.CLIMBING_START
					&& this.hanger.getState() == HangerState.NOT_HANGING) {
				this.hanger.setState(HangerState.WAITING_FOR_CYLINDERS);
			}

			if (this.driverIn.getHangerClimbButton() && this.hanger.getState() == HangerState.ARMS_UP) {
				this.hanger.setState(HangerState.HANGING);
			}

			if(this.hanger.getState() == HangerState.HANGING){
				if(this.driverIn.getShooterOffButton()){
					this.hanger.setState(HangerState.HANGER_ABORT);
				}
			}

			////////////////////////////
			///////// LED CODE /////////
			////////////////////////////

		}

		// this.led.setLed(this.desiredLedState);
		this.drive.calculate();
		this.shooter.calculate();
		this.intake.calculate();
		this.indexer.calculate();
		this.turret.calculate();
		this.hanger.calculate();

	}

	@Override
	public void disable() {
		this.drive.disable();
		this.shooter.disable();
		this.intake.disable();
		this.indexer.disable();
		this.turret.disable();
		this.hanger.disable();

	}
}
