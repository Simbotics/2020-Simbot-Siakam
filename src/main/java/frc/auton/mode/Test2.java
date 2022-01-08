package frc.auton.mode;

import frc.auton.AutonOverride;
import frc.auton.RobotComponent;
import frc.auton.drive.DriveWait;
import frc.auton.indexer.IndexerSetState;
import frc.auton.indexer.IndexerWait;
import frc.auton.shooter.ShooterSetShoot;
import frc.auton.util.AutonWait;
import frc.auton.util.AutonWaitUntilAutoTime;
import frc.subsystems.Indexer.IndexerState;

/**
 *
 * @author Michael
 */
public class Test2 implements AutonMode {

	@Override
	public void addToMode(AutonBuilder ab) {

        ab.addCommand(new ShooterSetShoot(3000, 20));

        ab.addCommand(new AutonWait(100));
        ab.addCommand(new IndexerSetState(IndexerState.FAST_SHOOTING));
        ab.addCommand(new AutonWaitUntilAutoTime(14500));
        ab.addCommand(new AutonOverride(RobotComponent.INDEXER));
        ab.addCommand(new IndexerSetState(IndexerState.POST_SHOOTING));
        ab.addCommand(new IndexerWait());

        ab.addCommand(new IndexerWait());
    }

}
