package frc.subsystems;

import frc.io.RobotOutput;

public class LEDStrip {

    private static LEDStrip instance;
    private RobotOutput robotOut;

    public enum LEDColourState {
        OFF,INTAKING_BALL,OUTTAKE_BALL,TURRET_AIMING,TURRET_AIMED,
        TURRET_NO_TARGET,SHOOTER_AT_SPEED,HANGING_STARTED,HANGING_DONE,SPINNER_STARTED,
        SPINNER_ROTATION_COMPLETE,SPINNER_POSITION,SPINNER_POSITION_COMPLETE
    }

    private LEDStrip() {
        this.robotOut = RobotOutput.getInstance();
    }

    public static LEDStrip getInstance() {
        if (instance == null) {
            instance = new LEDStrip();
        }
        return instance;
    }
    
    public void setLed(LEDColourState state) {
       switch(state){
            case OFF:
                this.robotOut.setLEDStrip(0);
                break;
            case INTAKING_BALL:
                this.robotOut.setLEDStrip(0);
                break;
            case OUTTAKE_BALL:
                this.robotOut.setLEDStrip(0);
                break;
            case TURRET_AIMING:
                this.robotOut.setLEDStrip(0);
                break;
            case TURRET_AIMED:
                this.robotOut.setLEDStrip(0);
                break;
             case TURRET_NO_TARGET:
                this.robotOut.setLEDStrip(0);
                break;
            case SHOOTER_AT_SPEED:
                this.robotOut.setLEDStrip(0);
                break;
            case HANGING_STARTED:
                this.robotOut.setLEDStrip(0);
                break;
            case HANGING_DONE:
                this.robotOut.setLEDStrip(0);
                break;
            case SPINNER_STARTED:
                this.robotOut.setLEDStrip(0);
                break;
            case SPINNER_POSITION:
                this.robotOut.setLEDStrip(0);
                break;
            case SPINNER_POSITION_COMPLETE:
                this.robotOut.setLEDStrip(0);
                break;
            case SPINNER_ROTATION_COMPLETE:
                this.robotOut.setLEDStrip(0);
                break;
            }
        }

}