package org.usfirst.frc3534.RobotBasic;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static WPI_TalonSRX frontLeftMotor;		//1
	public static WPI_TalonSRX centerLeftMotor;		//2
	private static WPI_TalonSRX backLeftMotor;		//3
	//spare talon srx								//4
	//spare talon srx								//5
	public static WPI_TalonSRX shooter;				//6
	//spare talon srx
	private static WPI_TalonSRX backRightMotor;		//8
	public static WPI_TalonSRX centerRightMotor;	//9
	public static WPI_TalonSRX frontRightMotor;		//10

	public static Spark blinkin;

	public static SpeedControllerGroup rightSideMotors;
	public static SpeedControllerGroup leftSideMotors;

	/** EXAMPLE public static DoubleSolenoid elevatorCylinderOne;		//first value -> PCM A, CHANNEL 0, 1 */


	public static AHRS navx;

	public static final double wheelBase_width = 36;
	public static final double robotMaxVeloctiy = 168; // inches per second
	public static final double minMoveSpeed = .375;

	// Wheel Encoder Calculations
	public static final int countsPerRevEncoders = 1440; // 1440 if plugged into talon. 360 if directly into the
															// roborio; just go with, it its weird
	public static final double wheelDiameter = 6; // measured in inches
	public static final double inchesPerCountMultiplier = wheelDiameter * Math.PI / countsPerRevEncoders;
	public static final double codesPer100MillisToInchesPerSecond = inchesPerCountMultiplier * 10;

	public static void init() {

		frontLeftMotor = new WPI_TalonSRX(1);
		frontLeftMotor.setNeutralMode(NeutralMode.Brake);
		frontLeftMotor.set(ControlMode.PercentOutput, 0);
		frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

		centerLeftMotor = new WPI_TalonSRX(2);
		centerLeftMotor.setNeutralMode(NeutralMode.Brake);
		centerLeftMotor.set(ControlMode.PercentOutput, 0);

		backLeftMotor = new WPI_TalonSRX(3);
		backLeftMotor.setNeutralMode(NeutralMode.Brake);
		backLeftMotor.set(ControlMode.PercentOutput, 0);

		shooter = new WPI_TalonSRX(6);
		shooter.set(ControlMode.PercentOutput, 0);

		backRightMotor = new WPI_TalonSRX((8));
		backRightMotor.setNeutralMode(NeutralMode.Brake);
		backRightMotor.set(ControlMode.PercentOutput, 0);

		centerRightMotor = new WPI_TalonSRX((9));
		centerRightMotor.setNeutralMode(NeutralMode.Brake);
		centerRightMotor.set(ControlMode.PercentOutput, 0);

		frontRightMotor = new WPI_TalonSRX(10);
		frontRightMotor.set(ControlMode.PercentOutput, 0);
		frontRightMotor.setNeutralMode(NeutralMode.Brake);
		frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		rightSideMotors = new SpeedControllerGroup(frontRightMotor, centerRightMotor, backRightMotor);
		leftSideMotors = new SpeedControllerGroup(frontLeftMotor, centerLeftMotor, backLeftMotor);
		
		blinkin = new Spark(1);

		navx = new AHRS(SPI.Port.kMXP);

	}

	public enum DelayToOff{
	
		/** the delay in seconds until the solenoids for certain ports turn off
		  *	below is just an example from 2019 
		  */

		elevator_stage1a(3.0),
		elevator_stage1b(3.0),
		elevator_stage2(3.0),
		elevator_floor(3.0),
		hatchPanelApparatus_collapsed(2.0),
		hatchIntake_hold(2.0),
		armExtend_extended(3.0),
		armLift_collapsed(2.0),
		armLift_mid(2.0),
		armLift_up(2.0);

		public double time;

		private DelayToOff(double time){

			this.time = time * 1000;

		}
	}

	public enum FunctionStateDelay{
	
		/** the delay in seconds between different states in the function switch statements
		  *	creates the wait time between cases in other words
		  *	below is just an example from 2019 (the examples should probably be removed 
		  *	when the next years copy of the code is made)
		  */

		cargoIntakeFloor_elevatorStage1A_to_armExtendExtended_rollerIntake(1.0),
		cargoShoot_shooterShoot_to_shooterStop(0.2),
		hatchPlace_hatchIntakeRelease_to_hatchPanelApparatusExtended(0.05), 
		hatchPlace_hatchPanelApparatusExtended_to_hatchPanelApparatusCollapsed(0.25),
		hatchPlace_hatchPanelApparatusCollapsed_to_hatchPlaceCompleted(3.0),
		xButtonReset_armLiftMid_to_armExtendCollapsed(1.0);

		public double time;

		private FunctionStateDelay(double time){

			this.time = time * 1000;

		}
	}

	public enum PowerOutput{
	
		/** the power output in percentage for the different actions in the functions for the motors
		  * for example, an intake motor at a constant percentage of power while button pressed
		  *	below is just an example from 2019 (the examples should probably be removed 
		  *	when the next years copy of the code is made)
		  */

		intake_roller_intake(0.85), 
		intake_roller_stop(0.0),
		shooter_shooter_intake(-0.5),
		shooter_shooter_shoot(1.0),
		shooter_shooter_stop(0.0);

		public double power;

		private PowerOutput(double power){

			this.power = power;

		}

	}
}
