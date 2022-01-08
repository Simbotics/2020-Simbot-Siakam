package frc.auton.indexer;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.RobotOutput;
import frc.subsystems.Indexer;
import frc.subsystems.Indexer.IndexerState;

public class IndexerSpinFor extends AutonCommand {

    private RobotOutput robotOut;
    private Indexer indexer;
    private int indexerSlotAmount;
    private double indexerMaxRPM;

    public IndexerSpinFor(int slotAmount, double maxRPM) {
        super(RobotComponent.INDEXER);
        this.robotOut = RobotOutput.getInstance();
        this.indexer = Indexer.getInstance();
        this.indexerSlotAmount = slotAmount;
        this.indexerMaxRPM = maxRPM;
    }

    @Override
    public void firstCycle() {
        this.indexer.setIndexerMaxRPM(this.indexerMaxRPM);
        this.indexer.setSlotSpinAmount(this.indexerSlotAmount);
        this.indexer.setState(IndexerState.SPINNING_FOR_SLOTS);
        this.indexer.setFirstCycleBool(true);
    }

    @Override
    // Sets the motor outputs
    public boolean calculate() {
        this.indexer.calculate();
        boolean isDone = this.indexer.getState() != IndexerState.SPINNING_FOR_SLOTS;
         if(isDone){
            this.robotOut.setIndexerElevator(0.0);
            this.robotOut.setIndexerSpinner(0.0);
         }
        return isDone;
    }

    @Override
    // When activated, stops the robot
    public void override() {
        this.robotOut.setIndexerSpinner(0.0);
        this.robotOut.setIndexerElevator(0.0);
    }

}
