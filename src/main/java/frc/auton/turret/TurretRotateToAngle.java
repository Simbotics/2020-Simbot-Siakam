package frc.auton.turret;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.RobotOutput;
import frc.subsystems.Turret;
import frc.subsystems.Turret.TurretState;

public class TurretRotateToAngle extends AutonCommand {

    private RobotOutput robotOut;
    private Turret turret;
    private double turretAngle;

    public TurretRotateToAngle(double angle){
        super(RobotComponent.TURRET);
        this.robotOut = RobotOutput.getInstance();
        this.turret = Turret.getInstance();
        this.turretAngle = angle;
    }

	@Override
	public void firstCycle() {
        this.turret.setTurretTargetAngle(this.turretAngle);
		this.turret.setState(TurretState.ROTATING_TO_ANGLE);
	}

	@Override
	// Sets the motor outputs
	public boolean calculate() {
		turret.calculate();
		return false;
	}

	@Override
	// When activated, stops the robot
	public void override() {
        this.robotOut.setTurretOutput(0.0);
	}

}
