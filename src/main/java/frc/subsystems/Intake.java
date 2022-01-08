package frc.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.RobotOutput;
import frc.io.SensorInput;

public class Intake extends Subsystem {

    private static Intake instance;
    private RobotOutput robotOut;
    private SensorInput sensorIn;

    private IntakeState intakeState = IntakeState.OFF_UP;

    private boolean firstCycleOfIntakeDown = true;
    private double timeAtIntakeDown = 0;
    private double timeSinceIntakeDown = 0;

    public enum IntakeState {
        OFF_DOWN,OFF_UP, REVERSE, CLIMBING_START,EXTENDING_DOWN_CLIMBING, INTAKING_DOWN, RETRACTING_IN, INTAKING_UP,EXTENDING_DOWN,
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
        this.robotOut = RobotOutput.getInstance();
        this.sensorIn = SensorInput.getInstance();
        this.firstCycle();
    }

    @Override
    public void firstCycle() {

    }

    @Override
    public void calculate() {
        if(this.intakeState != IntakeState.EXTENDING_DOWN && this.intakeState != IntakeState.EXTENDING_DOWN_CLIMBING){
            this.firstCycleOfIntakeDown = true;
            this.timeSinceIntakeDown = 0;
            this.timeAtIntakeDown = 0;
        }


        switch (this.intakeState) {
            case OFF_DOWN:
            this.robotOut.setIntakeOutput(0.0);
            this.robotOut.setIntakeWrist(true);
            break;
            case OFF_UP:
            this.robotOut.setIntakeOutput(0.0);
            this.robotOut.setIntakeWrist(false);
            break;
            case REVERSE:
            this.robotOut.setIntakeOutput(0.6);
            break;
            case INTAKING_DOWN:
            this.robotOut.setIntakeWrist(true);
            this.robotOut.setIntakeOutput(-0.8);
            break;
            case CLIMBING_START:
            this.robotOut.setIntakeWrist(true);
            this.robotOut.setIntakeOutput(0);
            break;
            case EXTENDING_DOWN:
            this.robotOut.setIntakeWrist(true);
            this.robotOut.setIntakeOutput(0.0);
            if(this.firstCycleOfIntakeDown){
                this.timeAtIntakeDown = System.currentTimeMillis();
                this.firstCycleOfIntakeDown = false;
            }
            this.timeSinceIntakeDown = System.currentTimeMillis() - this.timeAtIntakeDown;
            if(isExtendedDown()){
                this.intakeState = IntakeState.INTAKING_DOWN;
            }
            break;
            case EXTENDING_DOWN_CLIMBING:
            this.robotOut.setIntakeWrist(true);
            this.robotOut.setIntakeOutput(0.0);
            if(this.firstCycleOfIntakeDown){
                this.timeAtIntakeDown = System.currentTimeMillis();
                this.firstCycleOfIntakeDown = false;
            }
            this.timeSinceIntakeDown = System.currentTimeMillis() - this.timeAtIntakeDown;
            if(isExtendedDown()){
                this.intakeState = IntakeState.CLIMBING_START;
            }
            break;
            case INTAKING_UP:
            this.robotOut.setIntakeWrist(true);
            this.robotOut.setIntakeOutput(-1.0);
            case RETRACTING_IN:
            this.robotOut.setIntakeWrist(false);
            this.robotOut.setIntakeOutput(0.0);
            break;
            }
            
            SmartDashboard.putString("INTAKE_STATE", this.intakeState.toString());
    }

    @Override
    public void disable() {
        this.robotOut.setIntakeOutput(0.0);
    }

    public void setState(IntakeState state) {
        this.intakeState = state;
    }

    public IntakeState getState(){
        return this.intakeState;
    }

    public boolean isExtendedDown(){
        return this.timeSinceIntakeDown > 500;
    }

}
