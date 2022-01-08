/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.auton.mode;

import frc.auton.drive.DriveTurnToAngle;
import frc.auton.drive.DriveWait;

/**
 * Add your docs here.
 */
public class DriveTurnTest implements AutonMode {

    @Override
    public void addToMode(AutonBuilder ab) {
        ab.addCommand(new DriveTurnToAngle(180.0, 1.0, 1.0, 1000));
        ab.addCommand(new DriveWait());
    }
}
