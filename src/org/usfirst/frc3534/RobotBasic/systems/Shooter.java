package org.usfirst.frc3534.RobotBasic.systems;

import org.usfirst.frc3534.RobotBasic.Robot;
import org.usfirst.frc3534.RobotBasic.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Shooter extends SystemBase implements SystemInterface {

	private WPI_TalonSRX shooter = RobotMap.shooter;

	private final int SHOOT = 0;
	private final int INTAKE = 1;
	private final int STOP = 2;
	private final int[] states = { SHOOT, INTAKE, STOP };

	private double shooterPower = 0;

	private double[] buttonPowers = { 0.5, -0.5 };

	private double[] buttonTimes = { 0.5, 0.75 };

	ButtonProcess shooterButton;

	public Shooter() {

		this.setButtonProcess(buttonTimes, STOP, Robot.designatedLoopPeriod / 1000);
		this.setStates(states);

	}

	@Override
	public void process() {

		boolean[] buttons = { Robot.oi.getController1().getAButtonPressed(),
				Robot.oi.getController1().getBButtonPressed() };

		this.buttonProcess(buttons);

		switch (this.getState()) {

		/*
		 * for each case, type the name of the integer for each state in order,
		 * preferably
		 * 
		 * KEEP STOP CASE
		 */
		case SHOOT:

			shooterPower = buttonPowers[0];

			break;

		case INTAKE:

			shooterPower = buttonPowers[1];

			break;

		case STOP:

			shooterPower = 0;

			break;

		default:

			shooterPower = 0;

		}

		shooter.set(shooterPower);

	}
}