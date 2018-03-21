/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6411.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
		//////////////////////LimeLight////////////////////////
		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx0 = table.getEntry("tx0");
		NetworkTableEntry tx1 = table.getEntry("tx1");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry tv = table.getEntry("tv");
		NetworkTableEntry ledMode = table.getEntry("ledMode");
		NetworkTableEntry camMode = table.getEntry("camMode");
		
		/////////////////////auto vars/////////////////////////
		public double xll, v, xf, xi, errorH, PH, IH, DH, kP, kI, kD, errorX, PX, IX, DX, LastKnown, FMSside, drivingDone;
		public int Moment;
		/*The X's are for the distance PID control and the H's are for the course correction 
		 * or horizontal PID control
		 */
		
		String GameData;
		
		int countBoi;
		
		//////////////////auto chooser stuff////////////////////
		private static final String Left = "Left";
		private static final String Center = "Center";
		private static final String Right = "Right";
		private static final String Baseline = "Baseline and Cube";
		private String AutoPos;
		private SendableChooser<String> SPC = new SendableChooser<>();
		
		//private SendableChooser<String> Player = new SendableChooser<>();
		
		//////////////////////Joysticks////////////////////////
		Joystick Xbox1 = new Joystick(0);
		
		////////////////////DriveTrain////////////////////////
		Victor Right1 = new Victor(0);
		Victor Right2 = new Victor(1);
		Victor Left1 = new Victor(2);
		Victor Left2 = new Victor(3);
		
		SpeedControllerGroup leftboi = new SpeedControllerGroup(Left1, Left2);
		SpeedControllerGroup rightboi = new SpeedControllerGroup(Right1, Right2);
		
		DifferentialDrive ZoomBoi = new DifferentialDrive(leftboi, rightboi);
		
		////////////////////Cube Grabber//////////////////////
		Spark LeftArm = new Spark(4);
		Spark RightArm = new Spark(5);
		Spark Elevator = new Spark(6);
		
		////////////////////random variables//////////////////
		
	@Override
	public void robotInit() {
		Right1.setSafetyEnabled(true);
		Right2.setSafetyEnabled(true);
		Left1.setSafetyEnabled(true);
		Left2.setSafetyEnabled(true);
		
		Right1.setExpiration(.2);
		Right2.setExpiration(.2);
		Left1.setExpiration(.2);
		Left2.setExpiration(.2);
		
		SPC.addDefault("The Left", Left);
		SPC.addObject("The Center", Center);
		SPC.addObject("The Right", Right);
		SPC.addObject("Baseline Only", Baseline);
		SmartDashboard.putData("Auto choices", SPC);
		
		/////////////////////Cube Grabber/////////////////////
		LeftArm.enableDeadbandElimination(true);
		RightArm.enableDeadbandElimination(true);
		Elevator.enableDeadbandElimination(true);
		
		/////////////////////get switch/scale/////////////////////////
		GameData = DriverStation.getInstance().getGameSpecificMessage();
		}

	@Override
	public void autonomousInit() {
		AutoPos = SPC.getSelected();
		GameData = DriverStation.getInstance().getGameSpecificMessage();
		
		countBoi = 0;
		xf = 0;
		Moment  = 0;
		errorX = 0;
		
		LeftArm.set(0);
		RightArm.set(0);
			switch (GameData.charAt(0)) {
			case 'L':
				FMSside = -1;
				break;
			case 'R':
				FMSside = 1;
				break;
			}
		LastKnown = -FMSside;
	}

	@Override
	public void autonomousPeriodic() {
//		ZoomBoi.arcadeDrive(0,0);
//		countBoi ++;
//		intek(-.3);
//		SmartDashboard.putNumber("Distance", TargetDis(0));
		switch(Moment) {
		case 0:
			initMove();
			errorX = 0;
			errorH = 0;
			break;
		case 1:
			GotoSwitch();
			countBoi = 0;
			break;
		case 2:
			countBoi ++;
			spitIt();
			break;
		case 3:
			ZoomBoi.tankDrive(0, 0);
			Elevator.set(0);
			intek(0);
			break;
		}
//		PowerUPPP();
		SmartDashboard.putNumber("Moment", Moment);
	}

	@Override
	public void teleopPeriodic() {
		controllerInput();
		Elevator();
		ZoomBoi.tankDrive(-LeftStick, -RightStick);
		cubeIntake();
	}

	@Override
	public void testPeriodic() {
	}
	
	@Override
	public void disabledInit() {
		
	}
	//////////////////////////////////////////////////////////
	public void visionH() {
		//The H is for Horizontal
		kP = .625;
		kI = .40;
		kD = 0;
		
		xll = tx.getDouble(0);
		v = tv.getDouble(0);
		
		if(xll < 0) {
			LastKnown = -1;
		}
		else if(xll > 0) {
			LastKnown = 1;
		}
		
		if(v == 1) {
			xi = xll;	
			xi = xi/27;
			PH = xi;
			IH += xi * .02;
			DH = (xf - xi)/.02;
			
			errorH = (kP * PH) + (kI * IH) +(kD * DH);
		}
		else if(v != 1) {
			errorH = kP * LastKnown;
		}
		
		xf = xi;
	} 
	
	public double LogeTwo = Math.log(2);
	public double TargetDis(double Setpoint) {
		double Target1 = tx0.getDouble(0);
		double Target2 = tx1.getDouble(0);
		
		double Dis = Math.abs(Target1 - Target2);
		
		Dis = Dis + Setpoint;
		Dis = Dis / .58;
		Dis = Math.log(Dis)/LogeTwo;
		Dis = (-Dis)/0.0390947;
		return Dis;
	}
	public void GotoSwitch() {
		visionH();
		errorX = TargetDis(0);
		errorX = errorX/60;
		
		if(v == 0) {
			errorX = .55;
		}
		
		ZoomBoi.arcadeDrive(LimitX(errorX), errorH);
		
		SmartDashboard.putNumber("Horizonal Error", errorH);
		SmartDashboard.putNumber("Distance", errorX);
		
		if(Math.abs(errorX) > .1 && Math.abs(errorH) >= .3) {
		}
		else if(Math.abs(errorX) <= .21 && Math.abs(errorH) < .3) {
			Moment = 2;
		}
	}
	public double LimitX(double limit) {
		if(Math.abs(limit) > .3) {
		}
		else if(Math.abs(limit) <= .3) {
			limit = Math.copySign(.425, limit);
		}
		else if(Math.abs(limit) >= .85) {
			limit = Math.copySign(.8, limit);
		}
		return limit;
	}
	public void initMove() {
		switch(AutoPos) {
		case Left:
			if(FMSside == 1) {
				AutoPos = Baseline;
			}
			
			countBoi ++;
			PowerUPPP();
			if(countBoi <= 125) {
				
			}
			else if(countBoi > 125) {
				Moment = 1;
			}
			break;
		case Center:
			countBoi ++;
			PowerUPPP();
			centerBoi();
			if(countBoi <= 166) {
			}
			else if(countBoi > 166) {
				ZoomBoi.arcadeDrive(0, 0);
				intek(0);
				Moment = 1;
			}
			break;
		case Right:
			if(FMSside == -1) {
				AutoPos = Baseline;
			}
			
			countBoi ++;
			PowerUPPP();
			if(countBoi <= 125) {
			}
			else if(countBoi > 125) {
				Moment = 1;
			}
			break;
		case Baseline:
			countBoi ++;
			if(countBoi <= 166) {
				ZoomBoi.arcadeDrive(.6, 0);
			}
			else {
				ZoomBoi.arcadeDrive(0, 0);
				Moment = 3;
			}
			break;
		}
	}
	public void PowerUPPP() {
		if(countBoi < 50) {
			ZoomBoi.tankDrive(.5, .5);
		}
		else if(countBoi == 50) {
			ZoomBoi.tankDrive(0, 0);
		}
		/***ENDING COUNT VALUES::125***********/
	}
	public void centerBoi() { //50, 50, 15 cyc
		if(countBoi < 101 && countBoi > 51) {
			ZoomBoi.arcadeDrive(.7, FMSside * .71);
		}
		else if(countBoi < 151 && countBoi >= 101) {
			if(FMSside == -1) {
				ZoomBoi.tankDrive(.6, .6);
			}
			else {
			ZoomBoi.tankDrive(.33, .33);
			}
		}
		else if(countBoi < 166 && countBoi >= 151) {
			ZoomBoi.arcadeDrive(.45, -.45 * FMSside);
		}
		else if(countBoi == 166) {
			ZoomBoi.arcadeDrive(0, 0);
			intek(0);
		}
		
		/***ENDING COUNT VALUES::200***************/
	}	
	public void spitIt() {
		if(countBoi < 50 && countBoi >= 0) {
			ZoomBoi.tankDrive(.5, .5);
			Elevator.set(1);
		}
		else if(countBoi >= 50 && countBoi < 75) {
			ZoomBoi.tankDrive(0, 0);
			Elevator.set(.6);
		}
		else if(countBoi >= 75 && countBoi < 125) {
			LeftArm.set(.55);
			RightArm.set(-.55);
			Elevator.set(0);
		}
		else if(countBoi == 125) {
			ZoomBoi.tankDrive(0, 0);
			intek(0);
			Moment = 3;
		}
	}
	public void intek(double datboi) {
		LeftArm.set(datboi);
		RightArm.set(datboi);
	}
	
	////////////////////////////////////////////////////
	public double LeftStick, RightStick, LeftThrottle, RightThrottle;
	public boolean in, out;
	
	void controllerInput() {
		LeftStick = Xbox1.getRawAxis(1);
		RightStick = Xbox1.getRawAxis(5); //Driving
		
		in = Xbox1.getRawButton(6);
		out = Xbox1.getRawButton(5); //Intake
		
		LeftThrottle = Xbox1.getRawAxis(2);
		RightThrottle = Xbox1.getRawAxis(3); //Elevator
		
		RightThrottle = RightThrottle * .75; //Elevator
	}
	void cubeIntake() {
		if(in) {
			LeftArm.set(-1);
			RightArm.set(1);
		}
		
		else if(out) {
			LeftArm.set(1);
			RightArm.set(-1);
		}
		
		else{
			LeftArm.set(0);
			RightArm.set(0);
		}
}
	void Elevator() {
		if(LeftThrottle > .1) {
			Elevator.set(LeftThrottle); //up
		}
		else if(RightThrottle > .1) {
			Elevator.set(-RightThrottle);
		}
		else {
			Elevator.set(0);
		}
}
	
}
