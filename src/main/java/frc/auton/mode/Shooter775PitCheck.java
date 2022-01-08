package frc.auton.mode;

import frc.auton.AutonOverride;
import frc.auton.RobotComponent;
import frc.auton.shooter.ShooterConfigureMotors;
import frc.auton.shooter.ShooterSetSingleShoot;
import frc.auton.shooter.ShooterSetSingleShoot.ShooterMotor;
import frc.auton.util.AutonWait;

public class Shooter775PitCheck implements AutonMode {

@Override
public void addToMode(AutonBuilder ab){
    ab.addCommand(new ShooterSetSingleShoot(ShooterMotor.MASTER));
    ab.addCommand(new AutonWait(2000));
    ab.addCommand(new AutonOverride(RobotComponent.SHOOTER));
    ab.addCommand(new AutonWait(1000));
    ab.addCommand(new ShooterSetSingleShoot(ShooterMotor.SLAVE1));
    ab.addCommand(new AutonWait(2000));
    ab.addCommand(new AutonOverride(RobotComponent.SHOOTER));
    ab.addCommand(new AutonWait(1000));
    ab.addCommand(new ShooterSetSingleShoot(ShooterMotor.SLAVE2));
    ab.addCommand(new AutonWait(2000));
    ab.addCommand(new AutonOverride(RobotComponent.SHOOTER));
    ab.addCommand(new AutonWait(1000));
    ab.addCommand(new ShooterSetSingleShoot(ShooterMotor.SLAVE3));
    ab.addCommand(new AutonWait(2000));
    ab.addCommand(new AutonOverride(RobotComponent.SHOOTER));
    ab.addCommand(new ShooterConfigureMotors());
}



}