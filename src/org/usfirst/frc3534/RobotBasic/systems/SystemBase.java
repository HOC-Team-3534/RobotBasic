package org.usfirst.frc3534.RobotBasic.systems;

public class SystemBase implements SystemInterface {

	private int state;
	private int[] states;

	ButtonProcess buttonProcess;

	@Override
	public void process() {

	}

	@Override
	public void setState(int state) {

		this.state = states[state];

	}

	@Override
	public int getState() {

		return state;

	}

	@Override
	public void setStates(int[] states) {

		this.states = states;

	}

	@Override
	public void setButtonProcess(double[] seconds, int stopState, double designatedTimePeriod) {

		buttonProcess = new ButtonProcess(seconds, stopState, designatedTimePeriod);

	}

	@Override
	public void setButtonProcess(boolean[] finishes, int stopState, double designatedTimePeriod) {

		buttonProcess = new ButtonProcess(finishes, stopState, designatedTimePeriod);

	}

	@Override
	public void buttonProcess(boolean[] buttons) {

		state = buttonProcess.process(buttons);

	}
}
