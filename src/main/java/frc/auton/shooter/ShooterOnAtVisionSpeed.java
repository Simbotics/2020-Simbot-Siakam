package frc.auton.shooter;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.RobotOutput;
import frc.subsystems.Shooter;
import frc.subsystems.Shooter.ShooterState;

public class ShooterOnAtVisionSpeed extends AutonCommand {

    private RobotOutput robotOut;
    private Shooter shooter;

    public ShooterOnAtVisionSpeed(){
        super(RobotComponent.SHOOTER);
        this.robotOut = RobotOutput.getInstance();
        this.shooter = Shooter.getInstance();
    }
    
	@Override
	public void firstCycle() {
        this.shooter.setState(ShooterState.SHOOTING);
	}

	@Override
	// Sets the motor outputs
	public boolean calculate() {
        shooter.calculate();
        // System.out.println("Shooter At Speed: " + this.shooter.isShooterAtSpeedAndAimed());
        return false;//this.shooter.isShooterAtSpeedAndAimed();
	}

	@Override
	// When activated, stops the robot
	public void override() {
        this.robotOut.setShooterOutput(0.0);
	}

}
