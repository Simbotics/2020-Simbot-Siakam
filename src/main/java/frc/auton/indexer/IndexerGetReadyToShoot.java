package frc.auton.indexer;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.RobotOutput;
import frc.subsystems.Indexer;
import frc.subsystems.Indexer.IndexerState;

public class IndexerGetReadyToShoot extends AutonCommand {

    private RobotOutput robotOut;
    private Indexer indexer;

    public IndexerGetReadyToShoot() {
        super(RobotComponent.INDEXER);
        this.robotOut = RobotOutput.getInstance();
        this.indexer = Indexer.getInstance();
    }

    @Override
    public void firstCycle() {
        this.indexer.setFirstCycleBool(true);
        this.indexer.setState(IndexerState.LINING_UP_SLOT);
    }

    @Override
    // Sets the motor outputs
    public boolean calculate() {
        
        this.indexer.calculate();
        if(this.indexer.getState() == IndexerState.LINING_UP_SLOT && this.indexer.isIndexerSpinnerPIDDone()){
            this.indexer.setState(IndexerState.BRAKING_WITH_BALL_IN_PLACE);
            System.out.println("I am setting the indexer state to be breaking with ball in place");
        } 
        if(this.indexer.getLastCycleState() == IndexerState.BRAKING_WITH_BALL_IN_PLACE){
            System.out.println("The last cycle was breaking with ball in place.. I'm done this command");
            return true;
        }
        return false;
    }

    @Override
    // When activated, stops the robot
    public void override() {
        this.robotOut.setIndexerSpinner(0.0);
        this.robotOut.setIndexerElevator(0.0);
    }

}
