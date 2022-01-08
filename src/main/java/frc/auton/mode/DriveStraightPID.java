package frc.auton.mode;

import frc.auton.drive.DriveStraightAtVelocity;
import frc.auton.drive.DriveToPoint;
import frc.auton.drive.DriveWait;

public class DriveStraightPID implements AutonMode {

    @Override
    public void addToMode(AutonBuilder ab) {
        ab.addCommand(new DriveWait());
        ab.addCommand(new DriveStraightAtVelocity(0.8));
    }
}