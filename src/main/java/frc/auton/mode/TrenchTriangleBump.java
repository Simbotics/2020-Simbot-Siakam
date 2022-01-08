package frc.auton.mode;

import frc.auton.AutonOverride;
import frc.auton.RobotComponent;
import frc.auton.drive.DriveSetPosition;
import frc.auton.drive.DriveToPoint;
import frc.auton.drive.DriveWait;
import frc.auton.indexer.IndexerGetReadyToShoot;
import frc.auton.indexer.IndexerSetState;
import frc.auton.indexer.IndexerShootFor;
import frc.auton.indexer.IndexerWait;
import frc.auton.intake.IntakeSetState;
import frc.auton.shooter.ShooterSetShoot;
import frc.auton.shooter.ShooterSetState;
import frc.auton.shooter.ShooterWait;
import frc.auton.turret.TurretAimToVisionTarget;
import frc.auton.turret.TurretRotateToAngle;
import frc.auton.turret.TurretSetState;
import frc.auton.turret.TurretWait;
import frc.auton.util.AutonWait;
import frc.auton.util.AutonWaitUntilAutoTime;
import frc.auton.util.AutonWaitUntilXPosition;
import frc.robot.RobotConstants;
import frc.subsystems.Indexer.IndexerState;
import frc.subsystems.Intake.IntakeState;
import frc.subsystems.Shooter.ShooterState;
import frc.subsystems.Turret.TurretState;
import frc.auton.shooter.ShooterOnAtVisionSpeed;

public class TrenchTriangleBump implements AutonMode{

    @Override
    public void addToMode(AutonBuilder ab) {

        // intake 2 balls 
        ab.addCommand(new DriveSetPosition(0, 0, 90));
        ab.addCommand(new IndexerSetState(IndexerState.BRAKING_WITH_BALL_IN_PLACE));
        ab.addCommand(new TurretRotateToAngle(90.0));
        ab.addCommand(new DriveToPoint(0, -8.1, 90, 2.5, 7, 0.25, 15, 10000));
        ab.addCommand(new IntakeSetState(IntakeState.EXTENDING_DOWN));
        ab.addCommand(new DriveToPoint(0, -5.1, 90, 5,5, 0.75, 15, 10000));
        

        // drive to triangle and shoot 5 
        ab.addCommand(new DriveToPoint(17.15, 6.7, 0, 0, 13, 0.2, 15, 45, 10000));
        ab.addCommand(new AutonWait(1000));
        ab.addCommand(new ShooterSetShoot(RobotConstants.TRIANGLE_SHOT_RPM, RobotConstants.TRIANGLE_SHOT_HOOD_ANGLE));
        ab.addCommand(new AutonWaitUntilXPosition(12.5, true));
        ab.addCommand(new AutonOverride(RobotComponent.INTAKE));
        ab.addCommand(new IntakeSetState(IntakeState.OFF_DOWN));
        ab.addCommand(new AutonOverride(RobotComponent.TURRET));
        ab.addCommand(new TurretSetState(TurretState.VISION_CONTROL));
        ab.addCommand(new DriveWait());
        ab.addCommand(new AutonOverride(RobotComponent.INDEXER));
        ab.addCommand(new IndexerShootFor(6, 40,false));
        ab.addCommand(new IndexerWait());

        // grab 2 off bump 
        ab.addCommand(new AutonOverride(RobotComponent.TURRET));
        ab.addCommand(new TurretRotateToAngle(90.0));
        ab.addCommand(new AutonOverride(RobotComponent.SHOOTER));
        ab.addCommand(new ShooterSetShoot(4800, 35.5));
        ab.addCommand(new DriveToPoint(13.2, -7.3, 115, 0.5, 10, 0.5, 15, 60, 10000));
        ab.addCommand(new AutonOverride(RobotComponent.INTAKE));
        ab.addCommand(new IntakeSetState(IntakeState.INTAKING_DOWN));
        //ab.addCommand(new IndexerSetState(IndexerState.INTAKING));

        // drive and shoot 2 
        ab.addCommand(new DriveToPoint(17.2, -3.2, 0, 0,11, 0.1,15,90, 10000));
        ab.addCommand(new DriveWait());
        ab.addCommand(new AutonOverride(RobotComponent.TURRET));
        ab.addCommand(new AutonOverride(RobotComponent.INDEXER));
        ab.addCommand(new AutonOverride(RobotComponent.INTAKE));
        ab.addCommand(new IntakeSetState(IntakeState.OFF_DOWN));
        ab.addCommand(new TurretSetState(TurretState.VISION_CONTROL));
        ab.addCommand(new AutonOverride(RobotComponent.SHOOTER));
        ab.addCommand(new ShooterOnAtVisionSpeed());
        ab.addCommand(new IndexerSetState(IndexerState.LINING_UP_SLOT));
        ab.addCommand(new AutonWait(250));
        ab.addCommand(new AutonOverride(RobotComponent.INDEXER));
        //ab.addCommand(new IndexerShootFor(6, 40,true));
        ab.addCommand(new IndexerSetState(IndexerState.FAST_SHOOTING));
        ab.addCommand(new AutonWaitUntilAutoTime(14500));
        ab.addCommand(new AutonOverride(RobotComponent.INDEXER));
        ab.addCommand(new IndexerSetState(IndexerState.POST_SHOOTING));
        ab.addCommand(new IndexerWait());


       








        

    }
}