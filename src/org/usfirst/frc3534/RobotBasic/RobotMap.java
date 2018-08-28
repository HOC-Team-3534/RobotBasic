// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc3534.RobotBasic;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	public static WPI_TalonSRX frontRightMotor;
	private static WPI_TalonSRX backRightMotor;
	public static WPI_TalonSRX frontLeftMotor;
	private static WPI_TalonSRX backLeftMotor;
	
	public static WPI_TalonSRX shooter;
	
	public static SpeedControllerGroup rightSideMotors;
	public static SpeedControllerGroup leftSideMotors;
	
	public static AHRS navx;
	
	//Wheel Encoder Calculations
    private static final double countsPerRevEncoders = 1440; //1440 if plugged into talon. 360 if directly into the roborio; just go with, it its weird
    private static final double wheelDiameter = 6; //measured in inches
    public static final double inchesPerCountMultiplier = wheelDiameter * Math.PI / countsPerRevEncoders;

    public static void init() {

    	frontRightMotor = new WPI_TalonSRX(1);
    	frontRightMotor.set(ControlMode.PercentOutput, 1);
    	frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder , 0, 0);
    	
    	backRightMotor = new WPI_TalonSRX(2);
    	backRightMotor.set(ControlMode.PercentOutput, 1);
    	
    	frontLeftMotor = new WPI_TalonSRX(3);
    	frontLeftMotor.set(ControlMode.PercentOutput, 1);
    	frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder , 0, 0);
    	
    	backLeftMotor = new WPI_TalonSRX(4);
    	backLeftMotor.set(ControlMode.PercentOutput, 1);
    	
    	rightSideMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);
    	leftSideMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
    	
    	navx = new AHRS(SerialPort.Port.kMXP);
    	
    }
}