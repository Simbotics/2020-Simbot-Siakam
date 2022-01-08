package frc.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.imaging.SimLimelight;
import frc.imaging.SimLimelight.LimelightTargetType;
import frc.io.Dashboard;
import frc.io.RobotOutput;
import frc.io.SensorInput;
import frc.robot.RobotConstants;
import frc.util.SimPID;

public class Turret extends Subsystem {

    private static Turret instance;
    private RobotOutput robotOut;
    private SensorInput sensorIn;

    private double turretTargetAngle = 0;
    private double turretManualOutput = 0;

    private int cyclesAimed = 0;

    private TurretState currentTurretState = TurretState.OFF;

    private int cyclesInDeadZone = 0;//we might use this later but I'm not sure
    private boolean inDeadzone = false;

    public enum TurretState {
        ROTATING_TO_ANGLE, VISION_CONTROL, MANUAL, OFF, OPERATOR_POINTING
    }

    public static Turret getInstance() {
        if (instance == null) {
            instance = new Turret();
        }
        return instance;
    }

    private Turret() {
        this.robotOut = RobotOutput.getInstance();
        this.sensorIn = SensorInput.getInstance();
        this.firstCycle();
    }

    @Override
    public void firstCycle() {
        this.robotOut.configureTurretPID(RobotConstants.getTurretPID());
        this.currentTurretState = TurretState.OFF;
    }

    @Override
    public void calculate() {
        if(this.currentTurretState != TurretState.OPERATOR_POINTING 
        && this.currentTurretState != TurretState.ROTATING_TO_ANGLE
        && this.currentTurretState != TurretState.VISION_CONTROL){
            this.inDeadzone = false;
        }

        switch (this.currentTurretState) {
        case ROTATING_TO_ANGLE:

        if(this.turretTargetAngle > RobotConstants.TURRET_MAX_ANGLE){
            this.turretTargetAngle = RobotConstants.TURRET_MAX_ANGLE;
            this.inDeadzone = true;
        } else if(this.turretTargetAngle < RobotConstants.TURRET_MIN_ANGLE){
            this.turretTargetAngle = RobotConstants.TURRET_MIN_ANGLE;
            this.inDeadzone = true;
        } else {
            this.inDeadzone = false;
        }

            this.robotOut.setTurretPositionDegrees(this.turretTargetAngle);
            this.sensorIn.setLimelightTarget(LimelightTargetType.DRIVER);
            this.sensorIn.setPipeline(0);
            break;
        case VISION_CONTROL:
            this.sensorIn.setLimelightTarget(LimelightTargetType.HIGH_TARGET);
            this.sensorIn.setPipeline(1);

            if(this.sensorIn.getVisionTargetExists()){
                double theta = this.sensorIn.getTurretAngle() - this.sensorIn.getVisionTargetX();

                if(theta > RobotConstants.TURRET_MAX_ANGLE){
                    theta = RobotConstants.TURRET_MAX_ANGLE;
                    this.inDeadzone = true;
                } else if(theta < RobotConstants.TURRET_MIN_ANGLE){
                    theta = RobotConstants.TURRET_MIN_ANGLE;
                    this.inDeadzone = true;
                } else {
                    this.inDeadzone = false;
                }
                
                this.robotOut.setTurretPositionDegrees(theta);
                SmartDashboard.putNumber("Turret Vision Error", theta - this.sensorIn.getTurretAngle());
            }
            else{
                setTurretOutputMinMax(0);
            }
            break;
        case OPERATOR_POINTING:
            this.sensorIn.setPipeline(1);
            this.sensorIn.setLimelightTarget(LimelightTargetType.HIGH_TARGET);

            if(this.turretTargetAngle > RobotConstants.TURRET_MAX_ANGLE){
                this.turretTargetAngle = RobotConstants.TURRET_MAX_ANGLE;
                this.inDeadzone = true;
            } else if(this.turretTargetAngle < RobotConstants.TURRET_MIN_ANGLE){
                this.turretTargetAngle = RobotConstants.TURRET_MIN_ANGLE;
                this.inDeadzone = true;
            } else {
                this.inDeadzone = false;
            }

            this.robotOut.setTurretPositionDegrees(this.turretTargetAngle);
            

            double distance = Math.abs(this.turretTargetAngle - this.sensorIn.getTurretAngle());

            if(this.sensorIn.getVisionTargetExists() && distance < 55){
                this.currentTurretState = TurretState.VISION_CONTROL;
            }
            break;
        case MANUAL:
            this.setTurretOutputMinMax(this.turretManualOutput);
            this.sensorIn.setLimelightTarget(LimelightTargetType.DRIVER);
            this.sensorIn.setPipeline(0);
            break;
        case OFF:
            this.robotOut.setTurretOutput(0.0);
            this.sensorIn.setLimelightTarget(LimelightTargetType.DRIVER);
            this.sensorIn.setPipeline(0);
            break;
        default:
            this.robotOut.setTurretOutput(0.0);
            this.sensorIn.setLimelightTarget(LimelightTargetType.DRIVER);
            this.sensorIn.setPipeline(0);
        }
        SmartDashboard.putString("TURRET_STATE", this.currentTurretState.toString());
        SmartDashboard.putBoolean("TURRET IN DEADZONE: ", this.inDeadzone);
    }

    public void setState(TurretState state) {
        this.currentTurretState = state;
    }

    public void setTurretManualOutput(double output) {
        this.turretManualOutput = output;
    }

    public void setTurretTargetAngle(double angle) {
        this.turretTargetAngle = angle;
    }

    public TurretState getState(){
        return this.currentTurretState;
    }

    public void setTurretOutputMinMax(double output){
        if(this.sensorIn.getTurretAngle() < RobotConstants.TURRET_MAX_ANGLE && output > 0){
            this.robotOut.setTurretOutput(output);
        }else if(this.sensorIn.getTurretAngle() > RobotConstants.TURRET_MIN_ANGLE && output < 0){
            this.robotOut.setTurretOutput(output);
        } else{
            this.robotOut.setTurretOutput(0.0);
        }
    }

    public boolean isTurretAimedAtVisionTargetAuto() {
        if (this.sensorIn.getVisionTargetExists()) {
            if(this.sensorIn.getVisionTargetX() > -1.0 && this.sensorIn.getVisionTargetX() < 1.0){
                cyclesAimed++;
            } else {
                cyclesAimed = 0;
            }
        } else {
            cyclesAimed = 0;
        }
        return cyclesAimed > 5;
    }

    public boolean isTurretTargetInDeadzone(){
        return this.inDeadzone;
    }

    @Override
    public void disable() {
        this.robotOut.setTurretOutput(0.0);
    }
}