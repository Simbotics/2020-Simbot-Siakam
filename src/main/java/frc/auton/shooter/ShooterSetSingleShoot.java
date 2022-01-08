package frc.auton.shooter;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.RobotOutput;

public class ShooterSetSingleShoot extends AutonCommand {

    private RobotOutput robotOut;
    private ShooterMotor shooterMotor;

    public enum ShooterMotor {
        MASTER, SLAVE1, SLAVE2, SLAVE3
    }

    public ShooterSetSingleShoot(ShooterMotor motor) {// this is just copied from "ShooterSetShoot"
        super(RobotComponent.SHOOTER);
        this.robotOut = RobotOutput.getInstance();
        this.shooterMotor = motor;
    }

    @Override
    public void firstCycle() {
        this.robotOut.resetShooterMotors();
    }

    @Override
    // Sets the motor outputs
    public boolean calculate() {
        switch (this.shooterMotor) {
        case MASTER:
            this.robotOut.setShooterOutput(0.2);
            break;
        case SLAVE1:
            this.robotOut.setShooterSlave1Output(0.2);
            break;
        case SLAVE2:
            this.robotOut.setShooterSlave2Output(0.2);
            break;
        case SLAVE3:
            this.robotOut.setShooterSlave3Output(0.2);
            break;
        }
        return false;
    }

    @Override
    // When activated, stops the robot
    public void override() {
        this.robotOut.setShooterOutput(0.0);
        this.robotOut.setShooterSlave1Output(0.0);
        this.robotOut.setShooterSlave2Output(0.0);
        this.robotOut.setShooterSlave3Output(0.0);
    }

}
