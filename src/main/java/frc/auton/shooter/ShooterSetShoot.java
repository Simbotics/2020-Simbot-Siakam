package frc.auton.shooter;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.RobotOutput;
import frc.subsystems.Shooter;
import frc.subsystems.Shooter.ShooterState;

public class ShooterSetShoot extends AutonCommand {

    private RobotOutput robotOut;
    private Shooter shooter;
    private double shooterRPM;
    private double hoodAngle;

    public ShooterSetShoot(double shotRPM, double hoodAngle){
        super(RobotComponent.SHOOTER);
        this.robotOut = RobotOutput.getInstance();
        this.shooter = Shooter.getInstance();
        this.shooterRPM = shotRPM;
        this.hoodAngle = hoodAngle;
    }
    
	@Override
	public void firstCycle() {
        this.shooter.setAutoTargetRPM(this.shooterRPM);
        this.shooter.setAutoTargetHoodAngle(this.hoodAngle);
        System.out.println("my target RPM is " + this.shooterRPM);
        System.out.println("my target hood angle is " + this.hoodAngle);
        this.shooter.setState(ShooterState.AUTON_SHOOTING);
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
