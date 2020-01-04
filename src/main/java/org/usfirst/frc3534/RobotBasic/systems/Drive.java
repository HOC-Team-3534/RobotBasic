package org.usfirst.frc3534.RobotBasic.systems;

import org.usfirst.frc3534.RobotBasic.Robot;
import org.usfirst.frc3534.RobotBasic.RobotMap;
import org.usfirst.frc3534.RobotBasic.OI.Axes;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Drive extends SystemBase implements SystemInterface {

	private SpeedControllerGroup rightSide = RobotMap.rightSideMotors;
	private SpeedControllerGroup leftSide = RobotMap.leftSideMotors;
	private DifferentialDrive drive;

	private double rightPower, leftPower;

	private double deadband = 0.10;				//the deadband on the controller (forward/backward)
	private double turningDeadband = 0.20;		//the deadband on the controller (left/right)
	private boolean negative = false;

	private double yInput, xInput;				//used to store the direct input value of the respective axis on the controller
	private double yOut, xOut;					//used to save the percentage output calculated from the initial inputs

	private double left_command = 0.0, right_command = 0.0;		//used for autonomous power output

	private double last_error, distance_last_error;				//

	private double KpAim = 0.02; 				
	private double KdAim = 0.0010; 
	private double KpDistance = 0.02;
	private double KdDistance = .08;
	private double kpSkew = 0.003; 
	private double min_aim_command = 0.005;
	private double max_distance_command = 0.55; 
	private double max_side_to_side_correction = 0.2;

	public Drive() {

		drive = new DifferentialDrive(leftSide, rightSide);
		drive.setSafetyEnabled(true);
		drive.setMaxOutput(1.0);

	}

	@Override
	public void process() {

		if (Robot.teleop && Robot.enabled) {

			//Network 
			//Attempt at calling the Network Tables for Limelight and setting it 
			NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
			double tx = table.getEntry("tx").getDouble(0.0);
			double height = table.getEntry("tvert").getDouble(0.0);
			double skew = table.getEntry("ts").getDouble(0.0);
			double width = table.getEntry("thor").getDouble(0.0);

			/*if(Axes.DriverTargetMode.getAxis() >= 0.5){

				double heading_error = tx;
				double distance_error = 0.0;
				double steering_adjust = 0.0;
				double usableKpAim = 0.0;

				double sideToSideCorrection;

				if(skew >  -45){

					sideToSideCorrection = (-90 - (skew)) ;

				}else{

					sideToSideCorrection = (skew);

				}

				sideToSideCorrection = (90 + sideToSideCorrection) * kpSkew;

				if(height != 0.0){

					if(sideToSideCorrection <= 3){

						distance_error = 108.632 * Math.pow(.990, height);
	
					}else if(sideToSideCorrection <= 7){
	
						distance_error = 118.046 * Math.pow(.991, height);
	
					}else if(sideToSideCorrection <= 10){
	
						distance_error = 131.471 * Math.pow(.991, height);
	
					}else{
	
						distance_error = 122.591 * Math.pow(.9914, height);
	
					}

				}

				SmartDashboard.putNumber("Distance", distance_error);

				if(width > 425 && height < 130){

					distance_error = 20;

				}

				if(Math.abs(sideToSideCorrection) < 2){

					RobotMap.blinkin.set(0.77); //green

				}else if(sideToSideCorrection >= 2){

					RobotMap.blinkin.set(0.61); //red

				}else{

					RobotMap.blinkin.set(0.87); //blue

				}

				if(Math.abs(sideToSideCorrection) > 3 || sideToSideCorrection == 0) { //skew correction was 5 looking for outcome of calmer bot

					usableKpAim = KpAim * .3;

				} else {

					usableKpAim = KpAim;

				}

				if ( tx > 1.0 ) {

					steering_adjust = usableKpAim * heading_error + min_aim_command  + (heading_error - last_error) * KdAim ;
		
				}
				else if ( tx < -1.0 ) {

					steering_adjust = usableKpAim * heading_error - min_aim_command + (heading_error - last_error) * KdAim;
				
				}else{

					steering_adjust = 0.0;
					left_command = 0.0;
					right_command = 0.0;

				}

				last_error = heading_error;

				double distance_adjust = KpDistance * distance_error + KdDistance * (distance_error - distance_last_error);
				
				if(Math.abs(distance_adjust) > max_distance_command){

					distance_adjust = max_distance_command * Math.abs(distance_adjust) / distance_adjust;
					
				}

				distance_last_error = distance_error;

				left_command = steering_adjust + distance_adjust ;
				right_command = -steering_adjust + distance_adjust ;

				if(sideToSideCorrection > max_side_to_side_correction){

					sideToSideCorrection = max_side_to_side_correction;

				}

				if(skew > -45){

					left_command += sideToSideCorrection;
					right_command -= sideToSideCorrection;

				}else{

					right_command += sideToSideCorrection;
					left_command -= sideToSideCorrection;

				}

				drive.tankDrive(left_command, right_command);

			}else{*/

				RobotMap.blinkin.set(0.55); //color waves of team colors

				negative = false;
				yInput = Axes.Drive_ForwardBackward.getAxis();
				if(yInput < 0) negative = true;

				yOut = Math.abs(yInput);

				if(yOut > deadband){

					yOut -= deadband;
					yOut *= (1 / (1 - deadband));
					yOut = Math.pow(yOut, 2) * (1 - RobotMap.minMoveSpeed) + RobotMap.minMoveSpeed;
					if(negative) yOut = -yOut;

				}else{

					yOut = 0;

				}

				negative = false;
				xInput = Axes.Drive_LeftRight.getAxis();
				if(xInput < 0) negative = true;
				
				xOut = Math.abs(xInput);
				if(xOut > turningDeadband){

					xOut -= turningDeadband;
					xOut *= Math.pow((1 / (1 - turningDeadband)), 2);
					if(negative) xOut = -xOut;

				}else{

					xOut = 0;

				}

				if(Robot.oi.getController1().getTriggerAxis(Hand.kRight) >= 0.5){
					
					drive.arcadeDrive(yOut * 0.6, xOut * 0.5);

				}else{

					drive.arcadeDrive(yOut * 0.9, xOut * 1.0);

				}

			//}


		} else if (Robot.autonomous) {

			drive.tankDrive(leftPower, rightPower);

		}

		// uncomment the following code to test for max velocity
		/*
		 * double velocity;
		 * 
		 * if(RobotMap.frontLeftMotor.getSensorCollection().getQuadratureVelocity() >
		 * RobotMap.frontRightMotor.getSensorCollection().getQuadratureVelocity()) {
		 * 
		 * velocity =
		 * RobotMap.frontLeftMotor.getSensorCollection().getQuadratureVelocity();
		 * 
		 * }else{
		 * 
		 * velocity =
		 * RobotMap.frontRightMotor.getSensorCollection().getQuadratureVelocity();
		 * 
		 * }
		 * 
		 * SmartDashboard.putNumber("Velocity", velocity);
		 */

	}

	public void setRightPower(double power) {

		rightPower = power;

	}

	public void setLeftPower(double power) {

		leftPower = power;

	}

	public double getNavXAngle(){

		return RobotMap.navx.getAngle();

	}
}