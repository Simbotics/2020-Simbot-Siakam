package frc.auton.shooter;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.RobotOutput;

public class ShooterConfigureMotors extends AutonCommand {

    private RobotOutput robotOut;


    public ShooterConfigureMotors() {// this is just copied from "ShooterSetShoot"
        super(RobotComponent.SHOOTER);
        this.robotOut = RobotOutput.getInstance();
    }

    @Override
    public void firstCycle() {
        this.robotOut.resetShooterMotors();
    }

    @Override
    // Sets the motor outputs
    public boolean calculate() {
        this.robotOut.configureSpeedControllers();
        return true;
    }

    @Override
    // When activated, stops the robot
    public void override() {
    }

}
