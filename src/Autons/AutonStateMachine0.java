package Autons;

import org.usfirst.frc3534.RobotBasic.Robot;
import org.usfirst.frc3534.RobotBasic.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import PathfinderWorkArounds.EncoderFollower;
import PathfinderWorkArounds.Reader;
import PathfinderWorkArounds.Segment;

public class AutonStateMachine0 extends AutonStateMachineBase implements AutonStateMachineInterface {

	int state = 1;
	int stateCnt = 0;

	AHRS navX = RobotMap.navx;
	WPI_TalonSRX frontRight = RobotMap.frontRightMotor;
	WPI_TalonSRX frontLeft = RobotMap.frontLeftMotor;

	Segment[] rightTraj, leftTraj;

	EncoderFollower rightFollower, leftFollower;

	Reader reader;

	int step, posTraj = 1;
	/*
	 * step should equal the part that the trajectory as a whole is on. If it has
	 * five parts, and you are on the second part, make it equal to 2 for that case
	 * in the switch statement
	 * 
	 * posTraj should be made equal to the trajectory number from the trajectory
	 * visual/emulator. The value is named "posTraj" there as well.
	 */

	public AutonStateMachine0() {

		rightFollower.configurePIDVA(0.8, 0.0, 0.0, 1 / RobotMap.robotMaxVeloctiy, 0.0);
		leftFollower.configurePIDVA(0.8, 0.0, 0.0, 1 / RobotMap.robotMaxVeloctiy, 0.0);
		reader = new Reader();

	}

	@Override
	public void process() {

		int nextState = state;

		switch (state) {

		case 1:

			navX.zeroYaw();
			frontRight.getSensorCollection().setQuadraturePosition(0, 0);
			frontLeft.getSensorCollection().setQuadraturePosition(0, 0);

			nextState = 10;
			break;

		case 10:

			step = 1;

			rightTraj = reader.getSegments(Reader.Side.right, posTraj, step);
			leftTraj = reader.getSegments(Reader.Side.left, posTraj, step);

			rightFollower.setTrajectory(rightTraj);
			leftFollower.setTrajectory(leftTraj);

			rightFollower.configureEncoder(frontRight.getSensorCollection().getQuadraturePosition(),
					RobotMap.countsPerRevEncoders, RobotMap.wheelDiameter);
			leftFollower.configureEncoder(frontLeft.getSensorCollection().getQuadraturePosition(),
					RobotMap.countsPerRevEncoders, RobotMap.wheelDiameter);

			nextState = 20;
			break;

		case 20:

			Robot.drive
					.setRightPower(rightFollower.calculate(frontRight.getSensorCollection().getQuadraturePosition()));
			Robot.drive.setLeftPower(leftFollower.calculate(frontLeft.getSensorCollection().getQuadraturePosition()));

			if (rightFollower.isFinished() && leftFollower.isFinished()) {
				nextState = 100;
			}
			break;

		case 100:

			break;
		}

		if (nextState != state) {
			state = nextState;
			stateCnt = 0;
		} else {
			stateCnt++;
		}

	}

}
