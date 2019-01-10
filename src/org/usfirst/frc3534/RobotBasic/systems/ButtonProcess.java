package org.usfirst.frc3534.RobotBasic.systems;

public class ButtonProcess {

	boolean buttonNotPressed = true;

	int stateOut, stopState;

	double remainingTime = 0, designatedTimePeriod;

	double[] seconds;

	boolean[] finishes;

	boolean timeBased;

	int currentState;

	/**
	 * 
	 * @param seconds              the amount of time in seconds for the state
	 *                             before it returns to the stopState
	 * @param stopState            the stop state for the motor(s) being controlled
	 * @param designatedTimePeriod the time of the loop period of the robot
	 */
	public ButtonProcess(double[] seconds, int stopState, double designatedTimePeriod) {

		this.seconds = seconds;
		this.stopState = stopState;
		this.designatedTimePeriod = designatedTimePeriod;
		stateOut = this.stopState;
		timeBased = true;

	}

	/**
	 * 
	 * @param finishes             the boolean such as a sensor to check in order to
	 *                             return to the stopState
	 * @param stopState            the stop state for the motor(s) being controlled
	 * @param designatedTimePeriod the time of the loop period of the robot
	 */
	public ButtonProcess(boolean[] finishes, int stopState, double designatedTimePeriod) {

		this.finishes = finishes;
		this.stopState = stopState;
		this.designatedTimePeriod = designatedTimePeriod;
		stateOut = this.stopState;
		timeBased = false;

	}

	/**
	 * 
	 * @param buttons the joystick/xboxController whenPressed expression
	 */
	public int process(boolean[] buttons) {

		buttonNotPressed = true;

		if (timeBased) {

			if (stateOut == stopState) {

				for (int i = 0; i < buttons.length && buttonNotPressed; i++) {

					if (buttons[i]) {

						buttonNotPressed = false;
						stateOut = i;
						remainingTime = seconds[i];

					} else {

					}
				}

			} else {

				for (int i = 0; i < buttons.length && buttonNotPressed; i++) {

					if (buttons[i]) {

						if (stateOut == i) {

							buttonNotPressed = false;
							stateOut = stopState;
							remainingTime = 0;

						} else {

							buttonNotPressed = false;
							stateOut = i;
							remainingTime = seconds[i];

						}

					} else {

					}
				}

				remainingTime -= designatedTimePeriod;

				if (remainingTime <= 0) {

					remainingTime = 0;
					stateOut = stopState;

				} else {

				}
			}

		} else {

			if (stateOut == stopState) {

				for (int i = 0; i < buttons.length && buttonNotPressed; i++) {

					if (buttons[i]) {

						buttonNotPressed = false;
						stateOut = i;
						currentState = i;

					} else {

					}
				}

			} else {

				for (int i = 0; i < buttons.length && buttonNotPressed; i++) {

					if (buttons[i]) {

						if (stateOut == i) {

							buttonNotPressed = false;
							stateOut = stopState;

						} else {

							buttonNotPressed = false;
							stateOut = 0;

						}

					} else {

					}
				}

				if (finishes[currentState]) {

					stateOut = stopState;

				} else {

				}
			}
		}

		return stateOut;

	}
}