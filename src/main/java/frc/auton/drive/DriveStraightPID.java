package frc.auton.drive;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.Dashboard;
import frc.io.RobotOutput;
import frc.io.SensorInput;
import frc.robot.RobotConstants;
import frc.subsystems.Drive;
import frc.util.SimPID;


public class DriveStraightPID extends AutonCommand {

	private double distance;
    private Drive drive;
    private RobotOutput robotOut;
    private SensorInput sensorIn;
    SimPID driveStraightPID;

	public DriveStraightPID(double distance){
        super(RobotComponent.DRIVE);
        this.distance = distance;
        this.robotOut = RobotOutput.getInstance();
        this.sensorIn = SensorInput.getInstance();
        this.driveStraightPID  = new SimPID(RobotConstants.getDriveStraightPID());
    }

	@Override
	public void firstCycle() {
		driveStraightPID.setDesiredValue(this.distance);

	}

	@Override
	public boolean calculate() {
       double currentVelocity =  robotOut.getDriveFPSAverage();

       this.robotOut.setDriveLeft(this.driveStraightPID.calcPID(currentVelocity));
       this.robotOut.setDriveRight(this.driveStraightPID.calcPID(currentVelocity));
       return true;
    }

	@Override
	public void override() {

		this.robotOut.setDriveLeft(0);
		this.robotOut.setDriveRight(0);

	}

}
