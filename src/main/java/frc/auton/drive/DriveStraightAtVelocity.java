package frc.auton.drive;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.RobotOutput;
import frc.subsystems.Drive;

public class DriveStraightAtVelocity extends AutonCommand {

	private double velocity;
	private Drive drive;
	private RobotOutput robotOut;

	public DriveStraightAtVelocity(double velocity) {
		super(RobotComponent.DRIVE);
		this.velocity = velocity;

		this.drive = Drive.getInstance();
		this.robotOut = RobotOutput.getInstance();
		
		
	}

	@Override
	public void firstCycle() {
		this.drive.setTargetVelocity(this.velocity, this.velocity);
	}

	@Override
	// Sets the motor outputs
	public boolean calculate() {
		this.drive.calculate();
		
		return false;

	}

	@Override
	// When activated, stops the robot
	public void override() {
		this.robotOut.setDriveLeft(0.0);
		this.robotOut.setDriveRight(0.0);
	}

}
