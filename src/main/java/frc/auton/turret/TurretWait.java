package frc.auton.turret;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;

public class TurretWait extends AutonCommand {

	public TurretWait() {
		super(RobotComponent.TURRET);
	}

	@Override
	public boolean calculate() {
		return true;
	}

	@Override
	public void override() {
		// TODO Auto-generated method stub

	}

	@Override
	public void firstCycle() {
		// TODO Auto-generated method stub

	}

}