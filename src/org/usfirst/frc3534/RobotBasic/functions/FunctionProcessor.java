package org.usfirst.frc3534.RobotBasic.functions;

import org.usfirst.frc3534.RobotBasic.Robot;
import org.usfirst.frc3534.RobotBasic.systems.Arduino.LEDState;

public class FunctionProcessor{

    CargoIntakeFloor cargoIntakeFloor;
    CargoIntakeTop cargoIntakeTop;
    Elevate elevate;
    HatchPlace hatchPlace;
    HatchPickUp hatchPickUp;
    CargoShoot cargoShoot;
    //Compressor compressor;

    public FunctionProcessor(){

        cargoIntakeFloor = new CargoIntakeFloor();
        cargoIntakeTop = new CargoIntakeTop();
        elevate = new Elevate();
        hatchPlace = new HatchPlace();
        hatchPickUp = new HatchPickUp();
        cargoShoot = new CargoShoot();
        //compressor = new Compressor();

    }

    public void process(){

        Robot.arduino.setLEDState(LEDState.BLUE);
        cargoIntakeFloor.process();
        cargoIntakeTop.process();
        hatchPlace.process();
        hatchPickUp.process();
        cargoShoot.process();
        elevate.process();
        //compressor.process();

    }
}