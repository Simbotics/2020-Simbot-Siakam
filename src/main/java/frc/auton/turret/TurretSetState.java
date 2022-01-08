package frc.auton.turret;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.RobotOutput;
import frc.subsystems.Turret;
import frc.subsystems.Turret.TurretState;

public class TurretSetState extends AutonCommand {

    private RobotOutput robotOut;
    private Turret turret;
    private TurretState turretState;

    public TurretSetState(TurretState turretState){
        super(RobotComponent.TURRET);
        this.robotOut = RobotOutput.getInstance();
        this.turret = Turret.getInstance();
        this.turretState = turretState;
    }

	@Override
	public void firstCycle() {
		this.turret.setState(this.turretState);
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
