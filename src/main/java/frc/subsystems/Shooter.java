package frc.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.DriverInput;
import frc.io.RobotOutput;
import frc.io.SensorInput;
import frc.robot.RobotConstants;
import frc.util.SimBangBang;
import frc.util.SimPID;

public class Shooter extends Subsystem {

    private static Shooter instance;
    private RobotOutput robotOut;
    private SensorInput sensorIn;
    private DriverInput driverIn;

    private SimPID hoodPID;
    private double hoodTargetAngle = 0;

    private double lastTargetRPM = 4000;
    private double lastHoodTargetAngle = 20;

    private double targetRPM = 0;

    private double autoTargetRPM = 0;
    private double autoHoodTargetAngle = 0;

    private double RPMEpsilon = 50;
    private double hoodAngleEpsilon = 1.253;

    private boolean tuning = false;

    public enum ShooterState {
        OFF, HOOD_BACK_TO_ZERO, HOOD_GOING_TO_TARGET, UNDER_TRENCH, TRIANGLE_SHOT, SHOOTING, AUTON_SHOOTING
    }

    private ShooterState shooterState = ShooterState.OFF;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        this.robotOut = RobotOutput.getInstance();
        this.sensorIn = SensorInput.getInstance();
        this.driverIn = DriverInput.getInstance();
        this.hoodPID = new SimPID(RobotConstants.getHoodPID());
        SmartDashboard.putNumber("Shooter_Target_RPM", 3500);
        SmartDashboard.putNumber("HOOD_TARGET_ANGLE", 15);
        this.firstCycle();
    }

    @Override
    public void firstCycle() {
        this.robotOut.configureShooterPID(RobotConstants.getShooterPID());
        this.hoodPID.setConstants(RobotConstants.getHoodPID());
        this.hoodPID.setIRange(3);
        this.shooterState = ShooterState.OFF;
    }

    @Override
    public void calculate() {

        switch (this.shooterState) {
        case SHOOTING:
            if(!tuning){
                if (sensorIn.getVisionTargetExists()) {
                    this.targetRPM = shooterRPMBasedOnDistance(this.sensorIn.getVisionDistanceFeet());
                    this.hoodTargetAngle = hoodAngleBasedOnDistance(this.sensorIn.getVisionDistanceFeet());
                    this.lastTargetRPM = this.targetRPM;
                    this.lastHoodTargetAngle = this.hoodTargetAngle;
                } else {
                    this.targetRPM = this.lastTargetRPM;
                    this.hoodTargetAngle = this.lastHoodTargetAngle;
                }
                this.hoodPID.setDesiredValue(this.hoodTargetAngle);
                if (this.hoodPID.getDesiredVal() > 44.0) {
                    this.hoodTargetAngle = 44.0;
                    this.hoodPID.setDesiredValue(44.0);
                }
                if(this.targetRPM > RobotConstants.SHOOTER_MAX_RPM){
                    this.targetRPM = RobotConstants.SHOOTER_MAX_RPM;
                }
                robotOut.setShooterRPM(this.targetRPM);
                double outputshoot = this.hoodPID.calcPID(this.sensorIn.getHoodAngle());
                robotOut.setHood(outputshoot);
            } else {
                
                this.targetRPM = SmartDashboard.getNumber("Shooter_Target_RPM", 3500);
                this.hoodTargetAngle = SmartDashboard.getNumber("HOOD_TARGET_ANGLE", 15);
                this.hoodPID.setDesiredValue(this.hoodTargetAngle);
                if(this.hoodPID.getDesiredVal() > 44.0){
                    this.hoodTargetAngle = 44.0;
                    this.hoodPID.setDesiredValue(44.0);
                }
                if(this.targetRPM > RobotConstants.SHOOTER_MAX_RPM){
                    this.targetRPM = RobotConstants.SHOOTER_MAX_RPM;
                }
                this.robotOut.setHood(this.hoodPID.calcPID(this.sensorIn.getHoodAngle()));
                this.robotOut.setShooterRPM(this.targetRPM);
                // this.robotOut.setShooterOutput(1.0);
            }
            
            break;
        case UNDER_TRENCH:
            robotOut.setShooterOutput(0.0);
            robotOut.setHood(0.0);
            if (this.sensorIn.getHoodAngle() > 20) {
                this.hoodPID.setDesiredValue(20);
            }
            robotOut.setHood(this.hoodPID.calcPID(this.sensorIn.getHoodAngle()));
            break;
        case TRIANGLE_SHOT:
            this.targetRPM = RobotConstants.TRIANGLE_SHOT_RPM;
            this.hoodTargetAngle = RobotConstants.TRIANGLE_SHOT_HOOD_ANGLE;
            this.hoodPID.setDesiredValue(RobotConstants.TRIANGLE_SHOT_HOOD_ANGLE);
            if (this.hoodPID.getDesiredVal() > 44.0){
                this.hoodTargetAngle = 44.0;
                this.hoodPID.setDesiredValue(44.0);
            }
            if(this.targetRPM > RobotConstants.SHOOTER_MAX_RPM){
                this.targetRPM = RobotConstants.SHOOTER_MAX_RPM;
            }
            robotOut.setShooterRPM(RobotConstants.TRIANGLE_SHOT_RPM);
            robotOut.setHood(this.hoodPID.calcPID(this.sensorIn.getHoodAngle()));
            this.robotOut.setLowGear(false);
            break;
        case OFF:
            robotOut.setShooterOutput(0.0);
            this.hoodPID.setDesiredValue(0.0);
            if (this.hoodPID.getDesiredVal() > 44.0){
                this.hoodTargetAngle = 44.0;
                this.hoodPID.setDesiredValue(44.0);
            }
            if(this.targetRPM > RobotConstants.SHOOTER_MAX_RPM){
                this.targetRPM = RobotConstants.SHOOTER_MAX_RPM;
            }
            double outputoff = this.hoodPID.calcPID(this.sensorIn.getHoodAngle());
            robotOut.setHood(outputoff);
            this.lastHoodTargetAngle = 20;
            this.lastTargetRPM = 4000;
            break;
        case AUTON_SHOOTING:
            this.robotOut.setShooterRPM(this.autoTargetRPM);
            this.hoodPID.setDesiredValue(this.autoHoodTargetAngle);
            this.robotOut.setHood(this.hoodPID.calcPID(this.sensorIn.getHoodAngle()));
            break;
        default:
            robotOut.setShooterOutput(0.0);
            robotOut.setHood(0.0);
        }

        SmartDashboard.putString("SHOOTER_STATE", this.shooterState.toString());
        SmartDashboard.putBoolean("is Turret aimed: ", isTurretAimedAtVisionTarget());
        SmartDashboard.putBoolean("is shooter at speed :", isShooterAtSpeed());
        SmartDashboard.putBoolean("is hood aimed: ", isHoodAimed());
        SmartDashboard.putNumber("SHOOTER PERCENT OUTPUT", this.robotOut.getShooterOutput());
    }

    public void setState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    public ShooterState getState() {
        return this.shooterState;
    }

    public boolean isShooterAtSpeedAndAimed() {
        return this.sensorIn.getShooterRPM() < (this.targetRPM + this.RPMEpsilon)
                && this.sensorIn.getShooterRPM() > (this.targetRPM - this.RPMEpsilon)
                && this.sensorIn.getHoodAngle() < (this.hoodTargetAngle + this.hoodAngleEpsilon)
                && this.sensorIn.getHoodAngle() > (this.hoodTargetAngle - this.hoodAngleEpsilon);
    }

    public boolean isShooterAutoAtSpeedAndAimed() {
        return this.sensorIn.getShooterRPM() < (this.autoTargetRPM + this.RPMEpsilon)
                && this.sensorIn.getShooterRPM() > (this.autoTargetRPM - this.RPMEpsilon)
                && this.sensorIn.getHoodAngle() < (this.autoHoodTargetAngle + this.hoodAngleEpsilon)
                && this.sensorIn.getHoodAngle() > (this.autoHoodTargetAngle - this.hoodAngleEpsilon);
    }

    public boolean isTurretAimedAtVisionTarget() {
        if (this.sensorIn.getVisionTargetExists()) {
            return this.sensorIn.getVisionTargetX() > -1.0 && this.sensorIn.getVisionTargetX() < 1.0;
        }
        return true;
    }

    public boolean isShooterAtSpeed() {
        return this.sensorIn.getShooterRPM() < (this.targetRPM + this.RPMEpsilon)
                && this.sensorIn.getShooterRPM() > (this.targetRPM - this.RPMEpsilon);
    }

    public boolean isShooterAutoAtSpeed() {
        return this.sensorIn.getShooterRPM() < (this.autoTargetRPM + this.RPMEpsilon)
                && this.sensorIn.getShooterRPM() > (this.autoTargetRPM - this.RPMEpsilon);
    }

    public boolean isHoodAimed() {
        return this.sensorIn.getHoodAngle() < (this.hoodTargetAngle + this.hoodAngleEpsilon)
                && this.sensorIn.getHoodAngle() > (this.hoodTargetAngle - this.hoodAngleEpsilon);
    }

    public boolean isHoodAutoAimed() {
        return this.sensorIn.getHoodAngle() < (this.autoHoodTargetAngle + this.hoodAngleEpsilon)
                && this.sensorIn.getHoodAngle() > (this.autoHoodTargetAngle - this.hoodAngleEpsilon);
    }

    public void setTargetRPM(double targetRPM) {
        this.targetRPM = targetRPM;
    }

    public void setAutoTargetRPM(double RPM){
        this.autoTargetRPM = RPM;
    }

    public void setAutoTargetHoodAngle(double hoodAngle){
        this.autoHoodTargetAngle = hoodAngle;
    }

    public void setRPMEpsilon(double RPMEpsilon) {
        this.RPMEpsilon = RPMEpsilon;
    }

    public void setHoodAngle(double targetAngle) {
        this.hoodTargetAngle = targetAngle;
    }

    public double getTargetRPM() {
        return this.targetRPM;
    }

    public double hoodAngleBasedOnDistance(double distance) {
        return (23.6) + (0.818 * distance) + (0.00823 * distance * distance);
    }

    public double shooterRPMBasedOnDistance(double distance) {
        return/*(20.0 * distance) + 2000;*/ (106.5 * distance) + 3625;
    }

    @Override
    public void disable() {
        this.robotOut.setShooterOutput(0.0);
    }
}