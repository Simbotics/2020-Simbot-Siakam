package frc.auton.intake;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.RobotOutput;
import frc.subsystems.Drive;
import frc.subsystems.Intake;
import frc.subsystems.Drive.DriveState;
import frc.subsystems.Intake.IntakeState;

public class IntakeSetState extends AutonCommand {

    private RobotOutput robotOut;
    private Intake intake;
    private Intake.IntakeState intakeState;

    public IntakeSetState(Intake.IntakeState intakeState){
        super(RobotComponent.INTAKE);
        this.robotOut = RobotOutput.getInstance();
        this.intake = Intake.getInstance();
        this.intakeState = intakeState;
    }

	@Override
	public void firstCycle() {
		intake.setState(this.intakeState);
	}

	@Override
	// Sets the motor outputs
	public boolean calculate() {
		intake.calculate();
		return false;
	}

	@Override
	// When activated, stops the robot
	public void override() {
		this.robotOut.setIntakeOutput(0.0);
	}

}
