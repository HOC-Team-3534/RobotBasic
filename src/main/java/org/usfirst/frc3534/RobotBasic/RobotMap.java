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
	//spare talon srx								//2
	public static WPI_TalonSRX backLeftMotor;		//3
	//spare talon srx								//4
	//spare talon srx								//5
	public static WPI_TalonSRX shooter;				//6
	//spare talon srx
	public static WPI_TalonSRX backRightMotor;		//8
	//spare talon srx								//9
	public static WPI_TalonSRX frontRightMotor;		//10

	public static Spark blinkin;

	/** EXAMPLE public static DoubleSolenoid elevatorCylinderOne;		//first value -> PCM A, CHANNEL 0, 1 */


	public static AHRS navx;

	//public static final double wheelBase_width = 36;
	//public static final double robotMaxVeloctiy = 168; // inches per second
	public static final double minMoveSpeed = .375;
	public static final double maxVelocity = 5.0; //meters per second
	public static final double maxAngularVelocity = Math.PI; //radians per second

	// Wheel Encoder Calculations
	public static final double typicalAcceleration = 0.75; //meters per second per second
	public static final double robotMass = 18.15; //kg
	public static final double wheelDiameter = .1524; // measured in meters
	public static final double gearRatio = 13/93 * 25/50;
	public static final int ticksPerMotorRotation = 2048; // 2048 for Falcon500 (old ecnoders 1440 if in talon, 360 if into roboRIO)
	public static final double driveWheelTorque = (wheelDiameter / 2) * (robotMass / 4 * typicalAcceleration) * Math.sin(90);
	public static final double falconMaxRPM = 6380 - 1 / 0.0007351097 * driveWheelTorque;
	public static final double maxTicksPer100ms = falconMaxRPM * ticksPerMotorRotation / 60 / 10;
	public static final double distancePerMotorRotation = gearRatio * wheelDiameter * Math.PI;
	public static final double encoderVelocityToWheelVelocity =  1 / ticksPerMotorRotation * 10 * distancePerMotorRotation; //encoder ticks per 100ms to meters per second
	//public static final double inchesPerCountMultiplier = wheelDiameter * Math.PI / ticksPerRotation;
	//public static final double codesPer100MillisToInchesPerSecond = inchesPerCountMultiplier * 10;

	public static void init() {

		frontLeftMotor = new WPI_TalonSRX(1);
		frontLeftMotor.configFactoryDefault(30);
		frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
		frontLeftMotor.setSensorPhase(true);
		frontLeftMotor.setInverted(false);
		frontLeftMotor.configNominalOutputForward(0, 30);
		frontLeftMotor.configNominalOutputReverse(0, 30);
		frontLeftMotor.configPeakOutputForward(1, 30);
		frontLeftMotor.configPeakOutputReverse(-1, 30);
		frontLeftMotor.config_kF(0, 1023/maxTicksPer100ms, 30);
		frontLeftMotor.config_kP(0, 0.25, 30);
		frontLeftMotor.config_kI(0, 0, 30);
		frontLeftMotor.config_kD(0, 0, 30);

		backLeftMotor = new WPI_TalonSRX(3);
		backLeftMotor.configFactoryDefault(30);
		backLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
		backLeftMotor.setSensorPhase(true);
		backLeftMotor.setInverted(false);
		backLeftMotor.configNominalOutputForward(0, 30);
		backLeftMotor.configNominalOutputReverse(0, 30);
		backLeftMotor.configPeakOutputForward(1, 30);
		backLeftMotor.configPeakOutputReverse(-1, 30);
		backLeftMotor.config_kF(0, 1023/maxTicksPer100ms, 30);
		backLeftMotor.config_kP(0, 0.25, 30);
		backLeftMotor.config_kI(0, 0, 30);
		backLeftMotor.config_kD(0, 0, 30);

		shooter = new WPI_TalonSRX(6);
		shooter.set(ControlMode.PercentOutput, 0);

		backRightMotor = new WPI_TalonSRX(8);
		backRightMotor.configFactoryDefault(30);
		backRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
		backRightMotor.setSensorPhase(true);
		backRightMotor.setInverted(true);
		backRightMotor.configNominalOutputForward(0, 30);
		backRightMotor.configNominalOutputReverse(0, 30);
		backRightMotor.configPeakOutputForward(1, 30);
		backRightMotor.configPeakOutputReverse(-1, 30);
		backRightMotor.config_kF(0, 1023/maxTicksPer100ms, 30);
		backRightMotor.config_kP(0, 0.25, 30);
		backRightMotor.config_kI(0, 0, 30);
		backRightMotor.config_kD(0, 0, 30);

		frontRightMotor = new WPI_TalonSRX(10);
		frontRightMotor.configFactoryDefault(30);
		frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
		frontRightMotor.setSensorPhase(true);
		frontRightMotor.setInverted(true);
		frontRightMotor.configNominalOutputForward(0, 30);
		frontRightMotor.configNominalOutputReverse(0, 30);
		frontRightMotor.configPeakOutputForward(1, 30);
		frontRightMotor.configPeakOutputReverse(-1, 30);
		frontRightMotor.config_kF(0, 1023/maxTicksPer100ms, 30);
		frontRightMotor.config_kP(0, 0.25, 30);
		frontRightMotor.config_kI(0, 0, 30);
		frontRightMotor.config_kD(0, 0, 30);
		
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
