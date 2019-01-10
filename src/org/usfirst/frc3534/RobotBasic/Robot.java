package org.usfirst.frc3534.RobotBasic;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc3534.RobotBasic.systems.*;

import Autons.AutonStateMachine0;
import Autons.AutonStateMachine1;
import Autons.AutonStateMachine2;
import Autons.AutonStateMachine3;
import Autons.AutonStateMachineInterface;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static int AUTON_PERIODIC = 1;
	public static OI oi;
	public static Drive drive;
	public static Shooter shooter;

	private int loopPeriod = 0;
	private int loopCnt = 0;
	private int logCounter = 0;

	public static double designatedLoopPeriod = 20; // in milliseconds. milliseconds = seconds/1000. seconds to
													// milliseconds . seconds * 1000 = milliseconds

	public static boolean autonomous;
	public static boolean teleop;
	public static boolean enabled;

	private AutonStateMachineInterface autonStateMachine;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {

		RobotMap.init();

		drive = new Drive();
		shooter = new Shooter();

		// OI must be constructed after subsystems. If the OI creates Commands
		// (which it very likely will), subsystems are not guaranteed to be
		// constructed yet. Thus, their requires() statements may grab null
		// pointers. Bad news. Don't move it.
		oi = new OI();

	}

	/**
	 * This function is called when the disabled button is hit. You can use it to
	 * reset subsystems before shutting down.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void autonomousInit() {

		int desiredAutonMode = 0;

		try {

			desiredAutonMode = (int) SmartDashboard.getNumber("autonMode", 0);

		} catch (Exception ex) {
		}

		System.out.println("Running Auton " + desiredAutonMode);

		switch (desiredAutonMode) {

		case 0:

			autonStateMachine = new AutonStateMachine0();
			break;

		case 1:

			autonStateMachine = new AutonStateMachine1();
			break;

		case 2:

			autonStateMachine = new AutonStateMachine2();
			break;

		case 3:

			autonStateMachine = new AutonStateMachine3();
			break;

		}

		SmartDashboard.putNumber("aMode", desiredAutonMode);

	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {

		long prevLoopTime = 0;

		while (this.isAutonomous()) {

			RobotState("autonomous enabled");

			long currentTime = System.currentTimeMillis();

			if (currentTime - prevLoopTime >= designatedLoopPeriod) {

				log();

				loopPeriod = (int) (currentTime - prevLoopTime);
				prevLoopTime = currentTime;
				loopCnt++;

				// run processes
				drive.process();
				autonStateMachine.process();

			}

			Timer.delay(0.001);

		}

		RobotState("autonomous disabled");

	}

	@Override
	public void teleopInit() {

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {

		log();

		long prevLoopTime = 0;

		while (this.isOperatorControl() && this.isEnabled()) {

			log();

			RobotState("teleop enabled");

			long currentTime = System.currentTimeMillis();

			if (currentTime - prevLoopTime >= designatedLoopPeriod) {

				loopPeriod = (int) (currentTime - prevLoopTime);
				prevLoopTime = currentTime;
				loopCnt++;

				// run processes
				drive.process();
				shooter.process();

			}

			Timer.delay(0.001);

		}

		RobotState("teleop disabled");

	}

	public void log() {

		logCounter++;

		if (logCounter >= 5) {

			// SmartDashboard Numbers
			SmartDashboard.putNumber("Loop Period", loopPeriod);
			SmartDashboard.putNumber("Loop Count", loopCnt);
			SmartDashboard.putNumber("autonMode", 0);

			logCounter = 0;

		}
	}

	public void RobotState(String state) {

		switch (state) {

		case "teleop enabled":

			autonomous = false;
			teleop = true;
			enabled = true;
			break;

		case "teleop disabled":

			autonomous = false;
			teleop = true;
			enabled = false;
			break;

		case "autonomous enabled":

			autonomous = true;
			teleop = false;
			enabled = true;
			break;

		case "autonomous disabled":

			autonomous = true;
			teleop = false;
			enabled = false;
			break;

		}

	}
}