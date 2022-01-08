package frc.auton.indexer;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.RobotOutput;
import frc.subsystems.Indexer;

public class IndexerSetState extends AutonCommand {

    private RobotOutput robotOut;
    private Indexer indexer;
    private Indexer.IndexerState indexerState;

    public IndexerSetState(Indexer.IndexerState indexerState){
        super(RobotComponent.INDEXER);
        this.robotOut = RobotOutput.getInstance();
        this.indexer = Indexer.getInstance();
        this.indexerState = indexerState;
    }

	@Override
	public void firstCycle() {
		this.indexer.setState(indexerState);
	}

	@Override
	// Sets the motor outputs
	public boolean calculate() {
		indexer.calculate();
		return false;
	}

	@Override
	// When activated, stops the robot
	public void override() {
        this.robotOut.setIndexerSpinner(0.0);
        this.robotOut.setIndexerElevator(0.0);
	}

}
