package frc.auton.mode;

import frc.auton.AutonOverride;
import frc.auton.RobotComponent;
import frc.auton.drive.DriveSetOutput;
import frc.auton.drive.DriveSetPosition;
import frc.auton.drive.DriveStraightAtVelocity;
import frc.auton.drive.DriveToPoint;
import frc.auton.drive.DriveTurnToAngle;
import frc.auton.drive.DriveWait;
import frc.auton.indexer.IndexerSetState;
import frc.auton.indexer.IndexerSpinFor;
import frc.auton.indexer.IndexerWait;
import frc.auton.intake.IntakeSetState;
import frc.auton.shooter.ShooterHoldSpeed;
import frc.auton.shooter.ShooterOnAtVisionSpeed;
import frc.auton.shooter.ShooterSetState;
import frc.auton.turret.TurretSetState;
import frc.auton.util.AutonWait;
import frc.auton.util.TestPrint;
import frc.subsystems.Indexer;
import frc.subsystems.Indexer.IndexerState;
import frc.subsystems.Intake.IntakeState;
import frc.subsystems.Shooter.ShooterState;
import frc.subsystems.Turret.TurretState;

public class PreloadsPlusLineMiddle implements AutonMode {

    @Override
    public void addToMode(AutonBuilder ab) {
        ab.addCommand(new IndexerSetState(IndexerState.LINING_UP_SLOT));
        ab.addCommand(new TurretSetState(TurretState.VISION_CONTROL));
        ab.addCommand(new ShooterOnAtVisionSpeed());
        ab.addCommand(new IntakeSetState(IntakeState.EXTENDING_DOWN));
        ab.addCommand(new ShooterHoldSpeed());
        ab.addCommand(new AutonOverride(RobotComponent.INDEXER));
        ab.addCommand(new IndexerSpinFor(4, 40));
        ab.addCommand(new IndexerWait());
        
        ab.addCommand(new AutonOverride(RobotComponent.SHOOTER));
        ab.addCommand(new AutonOverride(RobotComponent.TURRET));
        ab.addCommand(new ShooterSetState(ShooterState.OFF));
        ab.addCommand(new ShooterHoldSpeed());
        ab.addCommand(new TurretSetState(TurretState.ROTATING_TO_ANGLE));
    

        // ab.addCommand(new DriveSetPosition(0, 0, 90));
        // ab.addCommand(new IntakeSetState(IntakeState.EXTENDING_OUT));
        // ab.addCommand(new DriveToPoint(4.25, -14.0, 90, 0, 13, 0.1, 15, 10000));
        // ab.addCommand(new DriveWait());
        // ab.addCommand(new IndexerSetState(Indexer.IndexerStates.SPIN180));
        // ab.addCommand(new DriveToPoint(0.0, -11.7, 75, 9, 13, 0.1, 15, 10000));
        // ab.addCommand(new DriveWait());
        // // ab.addCommand(new AutonOverride(RobotComponent.INDEXER));
        // // ab.addCommand(new IndexerSetState(Indexer.IndexerStates.OFF));
    
        // ab.addCommand(new DriveToPoint(4.0, -7.5, 35, 2, 9, 3.5, 20, 10000));
        
        // ab.addCommand(new DriveToPoint(-1.9, -9.0, 15, 0, 5, 0.1, 20, 10000));
        // ab.addCommand(new DriveWait());
        // ab.addCommand(new AutonWait(2000));
        // ab.addCommand(new AutonOverride(RobotComponent.INTAKE));
        // ab.addCommand(new IntakeSetState(IntakeState.OFF));
        // ab.addCommand(new TestPrint("DRIVE DONE"));

    }
}
