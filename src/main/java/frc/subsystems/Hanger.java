package frc.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.DriverInput;
import frc.io.RobotOutput;
import frc.io.SensorInput;
import frc.robot.RobotConstants;

public class Hanger extends Subsystem {

    private static Hanger instance;
    private RobotOutput robotOut;
    private SensorInput sensorIn;
    private DriverInput driverIn;

    public enum HangerState {
        NOT_HANGING, WAITING_FOR_INTAKE,ARMS_UP, HANGING, DONE_HANGING, MANUAL, WAITING_FOR_CYLINDERS, HANGER_ABORT
    }

    private HangerState hangerState = HangerState.NOT_HANGING;

    private boolean firstCycleOfClimb = true;
    private boolean firstCycelOfDoneHanging = true;
    private long timeOfDoneHanging = 0;
    private long deltaTimeOfDoneHanging = 0;
    private double runningPositionLeft = 0;
    private double runningPositionRight = 0;
    private double adjustmentRate = 500;
    private double timeAtTirstCycleOfCylinders = 0;
    private double deltaTimeOfCylinders = 0;
    private boolean firstCycleOfCylinders = true;

    public static Hanger getInstance() {
        if (instance == null) {
            instance = new Hanger();
        }
        return instance;
    }

    private Hanger() {
        this.robotOut = RobotOutput.getInstance();
        this.sensorIn = SensorInput.getInstance();
        this.driverIn = DriverInput.getInstance();
        this.firstCycle();
    }

    @Override
    public void firstCycle() {
        this.robotOut.configureHangerUpPID(RobotConstants.getHangerUpPID());
        this.robotOut.configureHangerDownPID(RobotConstants.getHangerDownPID());
        this.robotOut.setHangerPIDSlot(0);
    }

    @Override
    public void calculate() {

        if(this.hangerState != HangerState.WAITING_FOR_CYLINDERS 
            && this.hangerState != HangerState.ARMS_UP
            && this.hangerState != HangerState.HANGING
            && this.hangerState != HangerState.DONE_HANGING){
                this.firstCycleOfCylinders = true;
                this.timeAtTirstCycleOfCylinders = 0;
                this.deltaTimeOfCylinders = 0;
            }

        switch (this.hangerState) {
        case NOT_HANGING:
            robotOut.setClimberArms(false);
            robotOut.setClimberLock(true);
            robotOut.setHangerPosition(0.0);
            break;
        case WAITING_FOR_INTAKE:
            robotOut.setClimberArms(false);
            robotOut.setClimberLock(true);
            robotOut.setHangerPosition(0.0);
            break;
        
        case ARMS_UP:
            if (firstCycleOfClimb) {
                robotOut.resetClimbEncoders();
                firstCycleOfClimb = false;
            }
            robotOut.setClimberArms(true);
            robotOut.setClimberLock(false);

            double leftY = this.driverIn.getOperatorLeftY();
            double rightY = this.driverIn.getOperatorRightY();

           // SmartDashboard.putNumber("running pos left", this.runningPositionLeft);
           // SmartDashboard.putNumber("running pos right", this.runningPositionRight);

            if(leftY > 0.2){
                if(this.runningPositionLeft < RobotConstants.HANGER_HIGH_DISTANCE - RobotConstants.HANGER_MID_DISTANCE){ 
                    this.runningPositionLeft += leftY *adjustmentRate;
                }
            } else if(leftY < -0.2){
                if(this.runningPositionLeft > RobotConstants.HANGER_LOW_DISTANCE - RobotConstants.HANGER_MID_DISTANCE){ 
                    this.runningPositionLeft += leftY *adjustmentRate;
                }
            }

            if(rightY > 0.2){
                if(this.runningPositionRight < RobotConstants.HANGER_HIGH_DISTANCE - RobotConstants.HANGER_MID_DISTANCE){ 
                    this.runningPositionRight += rightY *adjustmentRate;
                }
            } else if(rightY < -0.2){
                if(this.runningPositionRight > RobotConstants.HANGER_LOW_DISTANCE - RobotConstants.HANGER_MID_DISTANCE){ 
                    this.runningPositionRight += rightY *adjustmentRate;
                }
            }

            this.robotOut.setLeftHangerPosition(RobotConstants.HANGER_MID_DISTANCE + this.runningPositionLeft);
            this.robotOut.setRightHangerPosition(RobotConstants.HANGER_MID_DISTANCE + this.runningPositionRight);
            
            break;
        case HANGING:
            //robotOut.setHangerPIDSlot(1);
            robotOut.setClimberArms(true);
            robotOut.setClimberLock(false);
            if (robotOut.getLeftHangerPosition() > RobotConstants.HANGER_CLIMB_PRESET) {
                robotOut.setLeftHangerOutput(-0.6);
                robotOut.setLeftHangerRampRate(1.5);
            } else {
                robotOut.setLeftHangerOutput(-0.2);
                robotOut.setLeftHangerRampRate(0.0);
            }
            if (robotOut.getRightHangerPosition() > RobotConstants.HANGER_CLIMB_PRESET) {
                robotOut.setRightHangerOutput(-0.6);
                robotOut.setRightHangerRampRate(1.5);
            } else {
                robotOut.setRightHangerOutput(-0.2);
                robotOut.setLeftHangerRampRate(0.0);
            }
            if(robotOut.getRightHangerPosition() < RobotConstants.HANGER_CLIMB_PRESET 
                && robotOut.getLeftHangerPosition() < RobotConstants.HANGER_CLIMB_PRESET){
                this.hangerState = HangerState.DONE_HANGING;
            }
           
            break;
        case DONE_HANGING:
            if (firstCycelOfDoneHanging) {
                this.timeOfDoneHanging = System.currentTimeMillis();
                this.firstCycelOfDoneHanging = false;
            }
            this.deltaTimeOfDoneHanging = System.currentTimeMillis() - this.timeOfDoneHanging;
            robotOut.setClimberArms(true);
            robotOut.setClimberLock(true);
            if (deltaTimeOfDoneHanging >= 300) {
                robotOut.setHangerOutput(0.0);
            } else {
                // robotOut.setHangerPosition(70000);
                robotOut.setHangerOutput(-0.2);
            }
            break;
        case MANUAL:
            robotOut.setLeftHangerOutput(this.driverIn.getOperatorLeftY());
            robotOut.setRightHangerOutput(this.driverIn.getOperatorRightY());

            robotOut.setLeftHangerRampRate(0.2);
            robotOut.setRightHangerRampRate(0.2);

            if (this.driverIn.getHangerArmsUpAndExtendButton()) {
                this.robotOut.setClimberArms(true);
            } else if (this.driverIn.getOperatorArmsDownButton()) {
                this.robotOut.setClimberArms(false);
            }

            if (this.driverIn.getOperatorRightLockButton() && this.driverIn.getOperatorLeftLockButton()) {
                robotOut.setClimberLock(false);
            } else {
                robotOut.setClimberLock(true);
            }
            break;
        case WAITING_FOR_CYLINDERS:
            if(this.firstCycleOfCylinders){
                this.timeAtTirstCycleOfCylinders = System.currentTimeMillis();
                this.deltaTimeOfCylinders = System.currentTimeMillis() - this.timeAtTirstCycleOfCylinders;
                this.firstCycleOfCylinders = false;
            }
            this.deltaTimeOfCylinders = System.currentTimeMillis() - this.timeAtTirstCycleOfCylinders;
            this.robotOut.setClimberArms(true);

            if(this.deltaTimeOfCylinders >= 500){
                this.hangerState = HangerState.ARMS_UP;
            }
            break;
        case HANGER_ABORT:
            this.robotOut.setClimberLock(true);
            this.robotOut.setClimberArms(true);
            this.robotOut.setLeftHangerRampRate(0.0);
            this.robotOut.setRightHangerRampRate(0.0);
            this.robotOut.setRightHangerOutput(0.0);
            this.robotOut.setLeftHangerOutput(0.0);
            break;
        default:

        }

        SmartDashboard.putString("HANGER_STATE", this.hangerState.toString());
    }

    public void setState(HangerState hangerState) {
        this.hangerState = hangerState;
    }

    public HangerState getState() {
        return this.hangerState;
    }

    private void setHangerHeight(double feet) {
        robotOut.setHangerPosition(feet * RobotConstants.HANGER_TICKS_PER_FOOT);
    }

    @Override
    public void disable() {
        robotOut.setHangerOutput(0.0);
    }
}
