package frc.auton.turret;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.RobotOutput;
import frc.subsystems.Turret;
import frc.subsystems.Turret.TurretState;

public class TurretAimToVisionTarget extends AutonCommand {

    private RobotOutput robotOut;
    private Turret turret;
    private double turretAngle;

    public TurretAimToVisionTarget(){
        super(RobotComponent.TURRET);
        this.robotOut = RobotOutput.getInstance();
        this.turret = Turret.getInstance();
    }

	@Override
	public void firstCycle() {
        this.turret.setTurretTargetAngle(this.turretAngle);
		this.turret.setState(TurretState.VISION_CONTROL);
	}

	@Override
	// Sets the motor outputs
	public boolean calculate() {
        turret.calculate();
        if(turret.isTurretAimedAtVisionTargetAuto()){
            robotOut.setTurretOutput(0.0);
            return true;
        }
		return false;
	}

	@Override
	// When activated, stops the robot
	public void override() {
        this.robotOut.setTurretOutput(0.0);
	}

}