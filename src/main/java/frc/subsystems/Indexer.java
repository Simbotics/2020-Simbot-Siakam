package frc.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.DriverInput;
import frc.io.RobotOutput;
import frc.io.SensorInput;
import frc.robot.RobotConstants;
import frc.util.SimPID;

public class Indexer extends Subsystem {

    private static Indexer instance;
    private RobotOutput robotOut;
    private SensorInput sensorIn;
    private DriverInput driverIn;

    private DigitalInput lightSensorBallOrArm;

    private boolean hasBeenZeroed = false;
    private double READY_TO_SHOOT_POSITION = 275;
    private double readyToShootAngleMin = 265;
    private double targetPosition = 0;
    private double desiredAbsAngle = 0;
    private boolean wasTargetInfrontOfCurrentLastCycle = false;
    private boolean passedZero = false;
    private boolean shootingFirstCycle = true;
    private long timeAtFirstCycle = 0;
    private long deltaTimeAtFirstCycle = 0;
    private int spinSlotAmount = 1;
    private double maxSpinningRPM = 40.0;
    private boolean spinForXSlotsFirstCycle = true;
    private long timePostShooting;
    private long deltaPostShootingTime;
    private boolean firstCyclePostShooting = false;

    private double angleMargin = 5.0;

    private SimPID spinnerPositionPID;

    // 25, 97, 169, 241, 313, these should be mapped out to correspond with the
    // slots
    private double[] ballAngles = { RobotConstants.INDEXER_BALL_SLOT_OFFSET,
            72.0 + RobotConstants.INDEXER_BALL_SLOT_OFFSET, 144.0 + RobotConstants.INDEXER_BALL_SLOT_OFFSET,
            216.0 + RobotConstants.INDEXER_BALL_SLOT_OFFSET, RobotConstants.INDEXER_BALL_SLOT_OFFSET + 288.0 };

    private enum BallState {
        BALL, EMPTY
    };

    private BallState[] indexerSlots = { BallState.BALL, BallState.BALL, BallState.BALL, BallState.BALL,
            BallState.BALL };// starts at closest to shooter, then count counterclockwise

    public enum IndexerState {// pre zeroing is for if we accidentally start looking in the middle of ball or
                               // arm
        HUMAN_LOADING, INTAKING, SHOOTING, OFF, ZEROING, PRE_ZEROING, SPIN180_FIRST_CYCLE, SPIN180, LINING_UP_SLOT,
        BRAKING_WITH_BALL_IN_PLACE, STICK_CONTROL, SPINNING_FOR_SLOTS, FAST_SHOOTING, POST_SHOOTING, UNJAM,
    };

    private IndexerState lastCycleState = IndexerState.BRAKING_WITH_BALL_IN_PLACE;

    private IndexerState indexerState = IndexerState.ZEROING;

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }
        return instance;
    }

    private Indexer() {
        this.robotOut = RobotOutput.getInstance();
        this.sensorIn = SensorInput.getInstance();
        this.driverIn = DriverInput.getInstance();
        this.lightSensorBallOrArm = new DigitalInput(0);
        this.spinnerPositionPID = new SimPID(RobotConstants.getIndexerSpinnerPosPID());
        this.timeAtFirstCycle = System.currentTimeMillis();
        this.firstCycle();
    }

    @Override
    public void firstCycle() {
        hasBeenZeroed = false;
       

        spinnerPositionPID = new SimPID(RobotConstants.getIndexerSpinnerPosPID());
        spinnerPositionPID.setMaxOutput(40.0);
        spinnerPositionPID.setMinDoneCycles(1);
        spinnerPositionPID.setIRange(10.0);
    }
      

    @Override
    public void calculate() {
       /* SmartDashboard.putString("INDEXER_STATE: ", this.indexerState.toString());
        SmartDashboard.putNumber("SLOT 0 POSITION", this.getCurrentAngleOfSlot(0));
        SmartDashboard.putNumber("SLOT 1 POSITION", this.getCurrentAngleOfSlot(1));
        SmartDashboard.putNumber("SLOT 2 POSITION", this.getCurrentAngleOfSlot(2));
        SmartDashboard.putNumber("SLOT 3 POSITION", this.getCurrentAngleOfSlot(3));
        SmartDashboard.putNumber("SLOT 4 POSITION", this.getCurrentAngleOfSlot(4));
        SmartDashboard.putNumber("CLOSEST SLOT TO SHOOTING POSITION", this.nextClosestSlot());
        */
        for (int i = 0; i < 5; i++) {
            detectBall(i);
        }

        //SmartDashboard.putNumber("Indexer spinner target angle", spinnerPositionPID.getDesiredVal());
       // SmartDashboard.putNumber("Indexer spinner error",
         //       Math.abs(spinnerPositionPID.getDesiredVal() - sensorIn.getIndexerPositionDegrees()));

        this.deltaTimeAtFirstCycle = System.currentTimeMillis() - this.timeAtFirstCycle;
        if (this.indexerState != IndexerState.SHOOTING && this.indexerState != IndexerState.SPINNING_FOR_SLOTS && this.indexerState != indexerState.FAST_SHOOTING) {
            this.shootingFirstCycle = true;
        }

        switch (this.indexerState) {
        case PRE_ZEROING:
            closeDoor();
            this.robotOut.setIndexerSpinnerNeutralMode(NeutralMode.Coast);
            this.robotOut.setIndexerWheel(true);
            if (!lightSensorBallOrArm.get()) {
                indexerState = IndexerState.ZEROING;
            }
            break;
        case ZEROING:
            closeDoor();
            this.robotOut.setIndexerSpinnerNeutralMode(NeutralMode.Coast);
            this.robotOut.setIndexerWheel(true);
            if (this.lightSensorBallOrArm.get()) {
                this.robotOut.resetIndexerEncoder();
                hasBeenZeroed = true;
            } else {
                setIndexerSpinnerRPM(0.05);
                setIndexerElevatorRPM(0.0);
            }
            break;
        case HUMAN_LOADING:
            closeDoor();
            this.robotOut.setIndexerSpinnerNeutralMode(NeutralMode.Coast);
            this.robotOut.setIndexerWheel(true);
            setIndexerSpinnerRPM(0.15);
            setIndexerElevatorRPM(0.0);
            break;
        case INTAKING:
            closeDoor();
            this.robotOut.setIndexerSpinnerNeutralMode(NeutralMode.Coast);
            this.robotOut.setIndexerWheel(true);
            setIndexerSpinnerRPM(25);
            setIndexerElevatorRPM(-450.0);//was -450
            break;
        case UNJAM:
            openDoor();
            this.robotOut.setIndexerWheel(false);
            setIndexerSpinnerRPM(0);
            setIndexerElevatorRPM(1700);
            break;
        case POST_SHOOTING:
            if(this.firstCyclePostShooting){
                this.timePostShooting = System.currentTimeMillis();
                this.firstCyclePostShooting = false;
            }
            this.deltaPostShootingTime = System.currentTimeMillis() - this.timePostShooting;

            if(this.deltaPostShootingTime < 500){
                openDoor();
                this.robotOut.setIndexerWheel(false);
                setIndexerSpinnerRPM(0);
                setIndexerElevatorRPM(1700);
                
            } else {
                this.indexerState = IndexerState.LINING_UP_SLOT;
            }


            break;
 
        case SHOOTING:
            openDoor();
            this.firstCyclePostShooting = true;
            this.robotOut.setIndexerSpinnerNeutralMode(NeutralMode.Coast);
            this.robotOut.setIndexerWheel(false);
            // if (isShotReady()) {
            // setIndexerSpinnerRPM(0.4);
            // } else {
            // setIndexerSpinnerRPM(0.25);
            // }
            if (this.shootingFirstCycle) {
                this.timeAtFirstCycle = System.currentTimeMillis();
                this.shootingFirstCycle = false;
            }
            if (this.deltaTimeAtFirstCycle > 200) {
                if (this.sensorIn.getVisionDistanceFeet() > 25.0) {
                    setIndexerSpinnerRPM(30);
                } else {
                    setIndexerSpinnerRPM(40);
                }
                setIndexerElevatorRPM(1700);
            } else {
                setIndexerSpinnerOutput(0.0);
                setIndexerElevatorOutput(0.0);
            }
            break;
        case FAST_SHOOTING:
            this.firstCyclePostShooting = true;
            openDoor();
            this.robotOut.setIndexerSpinnerNeutralMode(NeutralMode.Coast);
            this.robotOut.setIndexerWheel(false);
            // if (isShotReady()) {
            // setIndexerSpinnerRPM(0.4);
            // } else {
            // setIndexerSpinnerRPM(0.25);
            // }
            if (this.shootingFirstCycle) {
                this.timeAtFirstCycle = System.currentTimeMillis();
                this.shootingFirstCycle = false;
            }
            if (this.deltaTimeAtFirstCycle > 200) {
                if (this.sensorIn.getVisionDistanceFeet() > 25.0) {
                    setIndexerSpinnerRPM(50);
                } else {
                    setIndexerSpinnerRPM(60);
                }
                setIndexerElevatorRPM(1700);
            } else {
                setIndexerSpinnerOutput(0.0);
                setIndexerElevatorOutput(0.0);
            }
            break;
        case STICK_CONTROL:
            closeDoor();
            this.robotOut.setIndexerSpinnerNeutralMode(NeutralMode.Coast);
            setIndexerSpinnerRPM(Math.abs(this.driverIn.getOperatorRightX() * 80));
            this.robotOut.setIndexerWheel(true);
            setIndexerElevatorRPM(-450);
            break;
        case OFF:
            closeDoor();
            this.robotOut.setIndexerSpinnerNeutralMode(NeutralMode.Coast);
            this.robotOut.setIndexerWheel(true);
            setIndexerSpinnerRPM(0.0);
            setIndexerElevatorRPM(0.0);
            break;
        case SPIN180_FIRST_CYCLE:
            closeDoor();
            this.robotOut.setIndexerSpinnerNeutralMode(NeutralMode.Coast);
            this.robotOut.setIndexerWheel(true);
            targetPosition = (sensorIn.getIndexerPositionDegrees() + 180) % 360;
            setIndexerSpinnerPosition(targetPosition);
            indexerState = IndexerState.SPIN180;
            break;
        case SPIN180:
            this.robotOut.setIndexerWheel(true);
            this.robotOut.setIndexerSpinnerNeutralMode(NeutralMode.Coast);
            setIndexerSpinnerPosition(targetPosition);
            closeDoor();
            break;
        case LINING_UP_SLOT:
            this.robotOut.setIndexerWheel(true);
            this.robotOut.setIndexerSpinnerNeutralMode(NeutralMode.Coast);
            closeDoor();
            setIndexerElevatorOutput(0.0);
            if (howManyBalls() > 0) {
                moveSlotToShootingPosition(nextClosestSlot());
            }
            break;
        case BRAKING_WITH_BALL_IN_PLACE:
            this.robotOut.setIndexerWheel(true);
            this.robotOut.setIndexerSpinnerNeutralMode(NeutralMode.Brake);
            closeDoor();
            setIndexerSpinnerOutput(0.0);
            setIndexerElevatorOutput(0.0);
            break;
        case SPINNING_FOR_SLOTS:
            openDoor();
            this.robotOut.setIndexerSpinnerNeutralMode(NeutralMode.Coast);
            this.robotOut.setIndexerWheel(false);
            // if (isShotReady()) {
            // setIndexerSpinnerRPM(0.4);
            // } else {
            // setIndexerSpinnerRPM(0.25);
            // }
            if (this.shootingFirstCycle) {
                this.timeAtFirstCycle = System.currentTimeMillis();
                this.shootingFirstCycle = false;
            }
            if (this.deltaTimeAtFirstCycle > 200) {
                if (spinForXSlots(spinSlotAmount, maxSpinningRPM)) {
                    this.indexerState = IndexerState.LINING_UP_SLOT;
                }
                setIndexerElevatorRPM(1700);
            } else {
                setIndexerSpinnerOutput(0.0);
                setIndexerElevatorOutput(0.0);
            }

            break;
        default:
            this.robotOut.setIndexerWheel(true);
            this.robotOut.setIndexerSpinnerNeutralMode(NeutralMode.Coast);
            setIndexerSpinnerOutput(0.0);
            setIndexerElevatorOutput(0.0);
        }
        this.lastCycleState = this.indexerState;
        SmartDashboard.putString("INDEXER_STATE", this.indexerState.toString());
    }

    @Override
    public void disable() {
        setIndexerElevatorOutput(0.0);
        setIndexerSpinnerOutput(0.0);
    }

    private void setIndexerSpinnerOutput(double output) {
        this.robotOut.setIndexerSpinner(output);
    }

    private void setIndexerSpinnerRPM(double rpm) {
        this.robotOut.setIndexerSpinnerRPM(rpm);
    }

    private void setIndexerElevatorOutput(double output) {
        this.robotOut.setIndexerElevator(output);
    }

    private void setIndexerElevatorRPM(double rpm) {
        this.robotOut.setIndexerElevatorRPM(rpm);
    }

    // adds the necessary degress to get to the desired angle to the current value
    // of encoder ticks
    // Accumulative angular position of the indexer (no %360)
    private void setIndexerSpinnerPosition(double desiredAbsAngle) { // from 0 - 360
        double deltaAngle;
        if (sensorIn.getIndexerPositionDegrees() < desiredAbsAngle) { // target is infront
            deltaAngle = desiredAbsAngle - sensorIn.getIndexerPositionDegrees();
            wasTargetInfrontOfCurrentLastCycle = true;
        } else { // target is behind
            deltaAngle = 360 - (sensorIn.getIndexerPositionDegrees() - desiredAbsAngle);
            wasTargetInfrontOfCurrentLastCycle = false;
        }

        if (spinnerPositionPID.isDone()) {
            this.setState(IndexerState.BRAKING_WITH_BALL_IN_PLACE);
        }

        spinnerPositionPID.setDesiredValue(deltaAngle + sensorIn.getRunningIndexerPositionDegrees());
        double inputRPM = spinnerPositionPID.calcPID(sensorIn.getRunningIndexerPositionDegrees());
        robotOut.setIndexerSpinnerRPM(inputRPM);

    }

    public boolean spinForXSlots(int slotAmount, double maxRPM) {

        if (spinForXSlotsFirstCycle) {
            double deltaSlotAngle = 72 * slotAmount;
            spinForXSlotsFirstCycle = false;
            spinnerPositionPID.setDesiredValue(deltaSlotAngle + this.sensorIn.getRunningIndexerPositionDegrees());
        }

        //System.out.println(maxRPM);

        spinnerPositionPID.setMaxOutput(maxRPM);

        double indexerRPM = spinnerPositionPID.calcPID(this.sensorIn.getRunningIndexerPositionDegrees());
        this.robotOut.setIndexerSpinnerRPM(indexerRPM);
        boolean isDone = spinnerPositionPID.isDone();
       // System.out.println("is indexer pid done: " + isDone);
        return isDone;
    }

    public boolean getZeroedStatus() {
        return hasBeenZeroed;
    }

    private void detectBall(int slot) {
        if (this.sensorIn.getIndexerPositionDegrees() >= ballAngles[slot] - angleMargin
                && this.sensorIn.getIndexerPositionDegrees() <= ballAngles[slot] + angleMargin) { // checks to see if we
                                                                                                  // are in a range of
                                                                                                  // the
            // slot, then checks if theres a ball there
            if (lightSensorBallOrArm.get()) {
                indexerSlots[slot] = BallState.BALL;
            } else {
                indexerSlots[slot] = BallState.EMPTY;
            }
        }
    }

    private boolean isBallSlotReadyToBeShot(int slot) {
        if (getCurrentAngleOfSlot(slot) >= readyToShootAngleMin
                && getCurrentAngleOfSlot(slot) <= READY_TO_SHOOT_POSITION) {
            return true;
        }
        return false;
    }

    private boolean isShotReady() {
        return (isBallSlotReadyToBeShot(0) || isBallSlotReadyToBeShot(1) || isBallSlotReadyToBeShot(2)
                || isBallSlotReadyToBeShot(3) || isBallSlotReadyToBeShot(4));
    }

    public double getCurrentAngleOfSlot(int slot) {
        return (this.sensorIn.getIndexerPositionDegrees() + ballAngles[slot]) % 360;
        // return (ballAngles[slot]) % 360;
    }

    private void moveSlotToShootingPosition(int slot) {
        setIndexerSpinnerPosition((275 - ballAngles[slot]) % 360);// we wanna to put a slot to 275
    }

    private int nextClosestSlot() {
        int closestSlot = 0;
        double closestSlotAngleToShooter = Double.NEGATIVE_INFINITY;
        if (howManyBalls() == 0) {
            return -1;// kinda error tho if we pass this into ballAngles[] or
                      // moveSlotToShootingPosition
        }

        for (int i = 0; i < 5; i++) {
            double slotAngle = getCurrentAngleOfSlot(i);
            if (slotAngle < this.READY_TO_SHOOT_POSITION && slotAngle > closestSlotAngleToShooter) {
                closestSlot = i;
                closestSlotAngleToShooter = slotAngle;
            }
        }

        return closestSlot;
    }

    private int howManyBalls() {
        int count = 0;
        for (int i = 0; i < 5; i++) {
            if (indexerSlots[i].equals(BallState.BALL)) {
                count++;
            }
        }
        return count;
    }

    private void openDoor() {
        this.robotOut.setIndexerDoorClosed(false);
    }

    private void closeDoor() {
        this.robotOut.setIndexerDoorClosed(true);
    }

    public void setState(IndexerState state) {
        this.indexerState = state;
    }

    public boolean isIndexerSpinnerPIDDone() {
        return this.spinnerPositionPID.isDone();
    }

    public IndexerState getLastCycleState() {
        return this.lastCycleState;
    }

    public IndexerState getState() {
        return this.indexerState;
    }

    public void setSlotSpinAmount(int slotAmount) {
        this.spinSlotAmount = slotAmount;
    }

    public void setIndexerMaxRPM(double maxRPM) {
        this.maxSpinningRPM = maxRPM;
    }

    public void setFirstCycleBool(boolean True){
        this.shootingFirstCycle = True;
    }
}
