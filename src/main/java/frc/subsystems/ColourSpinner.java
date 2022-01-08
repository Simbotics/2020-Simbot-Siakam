package frc.subsystems;
import java.util.HashMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.SensorInput;
import frc.io.RobotOutput;
public class ColourSpinner extends Subsystem {
    private static ColourSpinner instance;
    private RobotOutput robotOut;
    private SensorInput sensorInput;
    private String startingColour = "Unknown";
    private String previousColour = "Unknown";
    private int rotationCounter = 0;
    private int coloursUntilPosition = 0;
    private HashMap<String, Integer> positionCounter = new HashMap<String, Integer>();
    public enum positionColours {
        RED, BLUE, YELLOW, GREEN, UNKNOWN
    }
    public enum ColourSpinnerState {
        PISTON_LOWERING, PISTON_RAISING, ROTATION_CONTROL_STARTED, ROTATION_CONTROL, ROTATION_CONTROL_SLOWING,
        POSITION_CONTROL, OFF
    }
    private ColourSpinnerState colourSpinnerState = ColourSpinnerState.PISTON_LOWERING;
    private positionColours detectedPositionColour = positionColours.UNKNOWN; 
    private positionColours desiredPositionColour = positionColours.UNKNOWN;

    public static ColourSpinner getInstance() {
        if (instance == null) {
            instance = new ColourSpinner();
        }
        return instance;
    }
    private ColourSpinner() {
        this.robotOut = RobotOutput.getInstance();
        this.sensorInput = SensorInput.getInstance();
        positionCounter.put("Red", 1);
        positionCounter.put("Yellow", 2);
        positionCounter.put("Blue", 3);
        positionCounter.put("Green", 4);
        this.firstCycle();
    }
    @Override
    public void firstCycle() {
        // TODO Auto-generated method stub
    }
    @Override
    public void calculate() {
        switch (this.colourSpinnerState) {
        case OFF:
            this.robotOut.setColourWheelOutput(0.0);
            break;
        case PISTON_LOWERING:
            this.robotOut.setColourWheelOutput(0.0);
            this.robotOut.setColourWheelUp(false);
            break;
        case PISTON_RAISING:
            this.robotOut.setColourWheelOutput(0.0);
            this.robotOut.setColourWheelUp(true);
            break;
        case ROTATION_CONTROL_STARTED:
            this.setStartingColour(this.sensorInput.getDetectedColourString());
            this.setPreviousColour(this.sensorInput.getDetectedColourString());
            this.setRotationCounter(0);
            this.setColourSpinnerState(this.colourSpinnerState.ROTATION_CONTROL);
            break;
        case ROTATION_CONTROL:
            this.robotOut.setColourWheelOutput(0.2);
            if (this.getPreviousColour() != this.sensorInput.getDetectedColourString()
                    && this.getStartingColour() == this.sensorInput.getDetectedColourString()) {
                this.setRotationCounter(++rotationCounter);
            }
            if (this.getRotationCounter() == 5) {
                this.setColourSpinnerState(colourSpinnerState.ROTATION_CONTROL_SLOWING);
            }
            break;
        case ROTATION_CONTROL_SLOWING:
            this.robotOut.setColourWheelOutput(0.05);
            if (this.getPreviousColour() != this.sensorInput.getDetectedColourString()
                    && this.getStartingColour() == this.sensorInput.getDetectedColourString()) {
                this.setRotationCounter(++rotationCounter);
            }
            if (this.getRotationCounter() == 6) {
                this.setColourSpinnerState(colourSpinnerState.PISTON_RAISING);
            }
            break;
        case POSITION_CONTROL:
            if (sensorInput.getDetectedColourString() == sensorInput.getPositionColour()) {
                this.setColourSpinnerState(colourSpinnerState.PISTON_RAISING);
            }
            for (int i = positionCounter.get(sensorInput.getDetectedColourString()); i != positionCounter
                    .get(sensorInput.getPositionColour()); i++) {
                if (i > 4) {
                    i = 1;
                }
                this.setColoursUntilPosition(coloursUntilPosition += 1);
            }
            this.robotOut.setColourWheelOutput(this.getColoursUntilPosition() * 0.05);
            break;
        default:
            this.robotOut.setColourWheelOutput(0.0);
            this.robotOut.setColourWheelUp(false);
        }
    }
    public void setDetectedPositionColour(positionColours detectedPositionColour) {
        this.detectedPositionColour = detectedPositionColour;
    }
    public void setDesiredPositionColour(positionColours desiredPositionColour) {
        this.desiredPositionColour = desiredPositionColour;
    }
    public void setColourSpinnerState(ColourSpinnerState colourSpinnerState) {
        this.colourSpinnerState = colourSpinnerState;
    }
    public ColourSpinnerState getColourSpinnerState() {
        return this.colourSpinnerState;
    }
    public void setStartingColour(String startingColour) {
        this.startingColour = startingColour;
    }
    public String getStartingColour() {
        return this.startingColour;
    }
    public void setPreviousColour(String previousColour) {
        this.previousColour = previousColour;
    }
    public String getPreviousColour() {
        return this.previousColour;
    }
    public void setRotationCounter(int rotationCounter) {
        this.rotationCounter = rotationCounter;
    }
    public int getRotationCounter() {
        return this.rotationCounter;
    }
    public void setColoursUntilPosition(int coloursUntilPosition) {
        this.coloursUntilPosition = coloursUntilPosition;
    }
    public int getColoursUntilPosition() {
        return this.coloursUntilPosition;
    }
    private double desiredColourBasedOutput(positionColours desiredPositionColour) {
        switch (desiredPositionColour) {
        case RED:
        default:
            return 0.0;
        }
    }
    @Override
    public void disable() {
        this.robotOut.setColourWheelOutput(0.0);
    }
}