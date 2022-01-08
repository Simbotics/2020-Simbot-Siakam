package frc.auton.indexer;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.RobotOutput;
import frc.subsystems.Indexer;
import frc.subsystems.Shooter;
import frc.subsystems.Indexer.IndexerState;
import frc.subsystems.Shooter.ShooterState;

public class IndexerShootFor extends AutonCommand {

    private RobotOutput robotOut;
    private Indexer indexer;
    private Shooter shooter;
    private int balls;
    private double indexerMaxRPM;

    private boolean startedShooting = false;
    private boolean usingVision;

    public IndexerShootFor(int balls, double maxRPM, boolean usingVision) {
        super(RobotComponent.INDEXER);
        this.robotOut = RobotOutput.getInstance();
        this.indexer = Indexer.getInstance();
        this.shooter = Shooter.getInstance();
        this.balls = balls;
        this.usingVision = usingVision;
        this.indexerMaxRPM = maxRPM;
    }

    @Override
    public void firstCycle() {
        this.indexer.setIndexerMaxRPM(this.indexerMaxRPM);
        this.indexer.setSlotSpinAmount(this.balls);
        this.indexer.setState(IndexerState.BRAKING_WITH_BALL_IN_PLACE);
        this.indexer.setFirstCycleBool(true);
    }

    @Override
    // Sets the motor outputs
    public boolean calculate() {
        this.indexer.calculate();
        if(usingVision){
            if(this.shooter.isShooterAtSpeedAndAimed() && this.indexer.getState() != IndexerState.SPINNING_FOR_SLOTS){
                this.indexer.setState(IndexerState.SPINNING_FOR_SLOTS);
                this.startedShooting = true;
            }    
        } else {
            if(this.shooter.isShooterAutoAtSpeedAndAimed() && this.indexer.getState() != IndexerState.SPINNING_FOR_SLOTS){
                this.indexer.setState(IndexerState.SPINNING_FOR_SLOTS);
                this.startedShooting = true;
            }    
        }
       
        boolean isDone = this.indexer.getState() == IndexerState.BRAKING_WITH_BALL_IN_PLACE && this.startedShooting;
        //  System.out.println("IndexerSpinFor currently done: " + isDone);
         if(isDone){
            this.robotOut.setIndexerElevator(0.0);
            this.robotOut.setIndexerSpinner(0.0);
            this.shooter.setState(ShooterState.OFF);
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
