package frc.auton.util;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;

import frc.subsystems.Drive;
import frc.subsystems.Hanger;
import frc.subsystems.Indexer;
import frc.subsystems.Intake;
import frc.subsystems.Shooter;
import frc.subsystems.Turret;


public class RunFirstCycle extends AutonCommand {

    private Drive drive;
    private Hanger hanger;
    private Indexer indexer;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    

    public RunFirstCycle() {
        super(RobotComponent.UTIL);
        this.drive = Drive.getInstance();
        this.hanger = Hanger.getInstance();
        this.indexer = Indexer.getInstance();
        this.intake = Intake.getInstance();
        this.shooter = Shooter.getInstance();
        this.turret = Turret.getInstance();

    }

    @Override
    public void firstCycle() {
        this.drive.firstCycle();
        this.hanger.firstCycle();
        this.indexer.firstCycle();
        this.intake.firstCycle();
        this.shooter.firstCycle();
        this.turret.firstCycle();

    }

    @Override
    public boolean calculate() {
        return true;
    }

    @Override
    public void override() {

    }

}
