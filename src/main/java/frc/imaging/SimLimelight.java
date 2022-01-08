/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.imaging;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.SensorInput;
import frc.robot.CoordinateConstants;
import frc.robot.RobotConstants;

/**
 * Add your docs here.
 */
public class SimLimelight {

    private static SimLimelight instance;
    private NetworkTable netWorkTable;

    public LimelightTargetType currentTargetType;

    public enum LimelightTargetType {
        DRIVER, HIGH_TARGET
    }

    private SimLimelight() {
        this.netWorkTable = NetworkTableInstance.getDefault().getTable("limelight");
        this.netWorkTable.getEntry("camera").setNumber(0);
        this.netWorkTable.getEntry("pipeline").setNumber(1);
    }

    public static SimLimelight getInstance() {
        if (instance == null) {
            instance = new SimLimelight();
        }
        return instance;
    }

    public void setLimelight(LimelightTargetType state) {
        currentTargetType = state;
    }

    public LimelightTargetType getLimelightState() {
        return currentTargetType;
    }

    public void setLEDMode(int ledMode) {// 0 is pipeline default, 1 is off, 2 is blink, 3 is on
        this.netWorkTable.getEntry("ledMode").setNumber(ledMode);
    }

    public void setPipeline(int pipeline){
        this.netWorkTable.getEntry("pipeline").setNumber(pipeline);
    }

    public double getTargetX() {
        return this.netWorkTable.getEntry("tx").getDouble(0); // because portrait
    }

    public double getTargetY() {
        return this.netWorkTable.getEntry("ty").getDouble(0); // because portrait
    }

    public boolean getTargetExists() {
        return this.netWorkTable.getEntry("tv").getDouble(0) == 1;
    }

    public double getTargetArea() {
        return this.netWorkTable.getEntry("ta").getDouble(0);
    }

    public double getTargetRotation() {
        return this.netWorkTable.getEntry("ts").getDouble(0);
    }

    public double[] getTargetPointsXY() {
        return this.netWorkTable.getEntry("tcornxy").getDoubleArray(new double[10]);
    }

    public double getVisionTargetAngle() { // uses trig to find angle
        double angle = 0;

        SmartDashboard.putNumber("Vision Target Angle", angle);
        return angle;
    }

}
