package org.usfirst.frc3534.RobotBasic.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc3534.RobotBasic.Robot;
import org.usfirst.frc3534.RobotBasic.RobotMap;
import org.usfirst.frc3534.RobotBasic.OI.Axes;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Drive extends SystemBase implements SystemInterface {

	private MecanumDrive drive;
	private WPI_TalonSRX frontLeft = RobotMap.frontLeftMotor, frontRight = RobotMap.frontRightMotor, backLeft = RobotMap.backLeftMotor, backRight = RobotMap.backRightMotor;

	private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
	private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
	private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
	private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

	private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
	private final MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, getAngle());

	private double rightPower, leftPower;

	private double deadband = 0.10;				//the deadband on the controller (forward/backward)
	private double turningDeadband = 0.20;		//the deadband on the controller (left/right)
	private boolean negative = false;

	private double longitudinal_input, latitudinal_input, rotational_input;			//used to store the direct input value of the respective axis on the controller
	private double longitudinal_output, latitudinal_output, rotational_output;		//used to save the percentage output calculated from the initial inputs

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

		drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
		
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
				longitudinal_input = Axes.Drive_ForwardBackward.getAxis();
				if(longitudinal_input < 0) negative = true;

				longitudinal_output = Math.abs(longitudinal_input);

				if(longitudinal_output > deadband){

					longitudinal_output -= deadband;
					longitudinal_output *= (1 / (1 - deadband));
					longitudinal_output = Math.pow(longitudinal_output, 2) * (1 - RobotMap.minMoveSpeed) + RobotMap.minMoveSpeed;
					if(negative) longitudinal_output = -longitudinal_output;

				}else{

					longitudinal_output = 0;

				}

				negative = false;
				latitudinal_input = Axes.Drive_LeftRight.getAxis();
				if(latitudinal_input < 0) negative = true;
				
				latitudinal_output = Math.abs(latitudinal_input);
				if(latitudinal_output > turningDeadband){

					latitudinal_output -= turningDeadband;
					latitudinal_output *= Math.pow((1 / (1 - turningDeadband)), 2);
					if(negative) latitudinal_output = -latitudinal_output;

				}else{

					latitudinal_output = 0;

				}

				rotational_input = Axes.Drive_Rotation.getAxis();
				rotational_output = rotational_input;

				if(Robot.oi.getController1().getTriggerAxis(Hand.kRight) >= 0.5){
					
					drive(longitudinal_output * 0.7 * RobotMap.maxVelocity, latitudinal_output * 0.7 * RobotMap.maxVelocity, rotational_output * RobotMap.maxAngularVelocity, true);

				}else{

					drive(longitudinal_output * RobotMap.maxVelocity, latitudinal_output * RobotMap.maxVelocity, rotational_output * RobotMap.maxAngularVelocity, true);

				}

			//}


		} else if (Robot.autonomous) {

			//drive.tankDrive(leftPower, rightPower);

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

	/**
   	* Method to drive the robot using joystick info.
   	*
   	* @param xSpeed        Speed of the robot in the x direction (forward).
   	* @param ySpeed        Speed of the robot in the y direction (sideways).
   	* @param rot           Angular rate of the robot.
   	* @param fieldRelative Whether the provided x and y speeds are relative to the field.
   	*/
  	@SuppressWarnings("ParameterName")
  	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    	var mecanumDriveWheelSpeeds = kinematics.toWheelSpeeds(
        	fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, getAngle()
        	) : new ChassisSpeeds(xSpeed, ySpeed, rot)
    	);
    	mecanumDriveWheelSpeeds.normalize(RobotMap.maxVelocity);
		setSpeeds(mecanumDriveWheelSpeeds);
		
  	}

	public MecanumDriveWheelSpeeds getCurrentState() {

		return new MecanumDriveWheelSpeeds(
			frontLeft.getSensorCollection().getQuadratureVelocity() * RobotMap.encoderVelocityToWheelVelocity,
			frontRight.getSensorCollection().getQuadratureVelocity() * RobotMap.encoderVelocityToWheelVelocity,
			backLeft.getSensorCollection().getQuadratureVelocity() * RobotMap.encoderVelocityToWheelVelocity,
			backRight.getSensorCollection().getQuadratureVelocity() * RobotMap.encoderVelocityToWheelVelocity
		);

	}

	public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
	
		frontLeft.set(ControlMode.Velocity, speeds.frontLeftMetersPerSecond / RobotMap.encoderVelocityToWheelVelocity);
		frontRight.set(ControlMode.Velocity, speeds.frontRightMetersPerSecond / RobotMap.encoderVelocityToWheelVelocity);
		backLeft.set(ControlMode.Velocity, speeds.rearLeftMetersPerSecond / RobotMap.encoderVelocityToWheelVelocity);
		backRight.set(ControlMode.Velocity, speeds.rearRightMetersPerSecond / RobotMap.encoderVelocityToWheelVelocity);

	}

	public void updateOdometry() {

		odometry.update(getAngle(), getCurrentState());

	}

	public void setRightPower(double power) {

		rightPower = power;

	}

	public void setLeftPower(double power) {

		leftPower = power;

	}

	public Rotation2d getAngle(){

		return Rotation2d.fromDegrees(-RobotMap.navx.getAngle());

	}
}