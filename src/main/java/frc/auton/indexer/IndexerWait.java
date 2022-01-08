package frc.auton.indexer;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;

public class IndexerWait extends AutonCommand {

	public IndexerWait() {
		super(RobotComponent.INDEXER);
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
