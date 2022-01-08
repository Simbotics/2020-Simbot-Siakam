package frc.auton.shooter;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.RobotOutput;
import frc.subsystems.Shooter;

public class ShooterHoldSpeed extends AutonCommand {

    private RobotOutput robotOut;
    private Shooter shooter;

    public ShooterHoldSpeed(){
        super(RobotComponent.SHOOTER);
        this.robotOut = RobotOutput.getInstance();
        this.shooter = Shooter.getInstance();
    }

	@Override
	public void firstCycle() {

	}

	@Override
	// Sets the motor outputs
	public boolean calculate() {
		shooter.calculate();
		return false;
	}

	@Override
	// When activated, stops the robot
	public void override() {
        this.robotOut.setShooterOutput(0.0);
	}

}
