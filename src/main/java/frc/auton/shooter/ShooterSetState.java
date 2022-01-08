package frc.auton.shooter;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.RobotOutput;
import frc.subsystems.Shooter;

public class ShooterSetState extends AutonCommand {

    private RobotOutput robotOut;
    private Shooter shooter;
    private Shooter.ShooterState shooterState;

    public ShooterSetState(Shooter.ShooterState shooterState){
        super(RobotComponent.SHOOTER);
        this.robotOut = RobotOutput.getInstance();
        this.shooter = Shooter.getInstance();
        this.shooterState = shooterState;
    }
    
	@Override
	public void firstCycle() {
        this.shooter.setState(this.shooterState);
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
