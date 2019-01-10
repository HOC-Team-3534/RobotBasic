package org.usfirst.frc3534.RobotBasic.systems;

public interface SystemInterface {

	public void process();

	public void setState(int state);

	public int getState();

	public void setStates(int[] states);

	/**
	 * 
	 * @param seconds              the amount of time in seconds for the state
	 *                             before it returns to the stopState
	 * @param stopState            the stop state for the motor(s) being controlled
	 * @param designatedTimePeriod the time of the loop period of the robot
	 */
	public void setButtonProcess(double[] seconds, int stopState, double designatedTimePeriod);

	/**
	 * 
	 * @param finishes             the boolean such as a sensor to check in order to
	 *                             return to the stopState
	 * @param stopState            the stop state for the motor(s) being controlled
	 * @param designatedTimePeriod the time of the loop period of the robot
	 */
	public void setButtonProcess(boolean[] finishes, int stopState, double designatedTimePeriod);

	/**
	 * 
	 * @param buttons the joystick/xboxController whenPressed expression
	 */
	public void buttonProcess(boolean[] buttons);

}
