package com.stan;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.nxt.Motor;
import lejos.robotics.RegulatedMotor;
import javax.bluetooth.RemoteDevice;

import lejos.nxt.Battery;
import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;
import lejos.nxt.comm.RConsole;
import lejos.util.Delay;

public class MountPoint2 extends Thread {
	private static enum MPST	{MPPREINIT, MPINIT, MPCONNG, MPCONN, MPCONFG, MPCONF, MPCALG, MPCAL, MPATTG, 
								 MPPARKED, MPRAISING, MPSTOPPED, MPSTARTING, MPORBITING, MPSTOPPING, MPLOWER, MPHALT,
								 MPBATT90, MPBATT75, MPBATT50, MPBATT25, MPBATT0}; 
	// should this be in a different class?
	private static enum CTRLST  {CTRLINIT, PARKED, RAISING, STOPPED, STARTING, ORBITING, STOPPING, LOWERING, HALT};  // valid states for PC controller

	private static int				delay   = 0;			// used for debugging purposes
	private static int				whoAmI  = 0;			// which NXT is this one?
	private static RegulatedMotor 	myMotor = Motor.C;		
	private static float			currentRadius;			// the current size of the orbit, which can change over time
	
	// speed variables
	private static float 			spoolRadius	= 1.27324F;	// radius of line spool in inches
	private static float			gearRatio   = (float) 24.0F/40.0F;		// ratio of gearing from motor speed to rotational speed of spool
	private static float			parkedLen	= 0.0F;		// keep track of how much line to extend
	private static float			stoppedLen	= 0.0F;		// keep track of how much line to extend
	private static int				defaultSpeed = 400;		// speed for calibration, raise/lower
	
	// battery variables
	private static float			maxBattery	= 9.0F;		// maximum voltage
	private static float			currBattery = 9.0F;		// assumed to start
	
	// state variables
	private static CTRLST 			orbiterState;		// current state of the orbiter
	private static MPST				mountPointState;	// current state of the mount point
	private static MPST				batteryState;		// bogus, but it should work
	private static boolean			readyToAttach;
	
	// controlling threads
	private static ControllerStateMonitor 	gcs;
	private static StartStopManager 		ss; 
	private static OrbitManager				orbMgr;
	
	// communication variables
	private static BTConnection 	ctrlConn;			// connection for controller
	private static DataInputStream 	ctrlInput;			// input stream to receive messages from the controller
	private static DataOutputStream ctrlOutput; 		// output stream to send messages to the controller
	
	// system configuration parameters
	private	static float			heightAboveGround, mountPointSpacing, orbiterDipSpacing;	// must be big enough that orbit is within mount points	
	private static int				startStopTime;			// amount of time it should take to start or stop, in seconds
	private static int				timeToOrbit;			// amount of time to complete one orbit, in seconds
	private static float			orbitRadius;			// how large the orbit should be when orbiting
	private static double			mountDistance;			// distance from mount point to orbit, in inches
															// must be big enough that orbit is within mount points - we will calculate it
	private static float			timeOffset;
	
	// other parameters
	private static float			orbiterHeight = 3.25F;			// how high about ground we attach the orbiter
	private static float			orbiterWidth  = 0.5F;			// how far from centerpoint we attach orbiter
	
	// constructor
	public MountPoint2() {
		// we are currently in the mount point initialization state
		// set the state condition appropriately
		mountPointState = MPST.MPPREINIT;	
		
		// figure out which NXT we are
		whoAmI = Integer.parseInt(Bluetooth.getFriendlyName().substring(3,4));
		LCD.clear();
		LCD.drawString("I am #"+whoAmI, 2, 3);
		Delay.msDelay(1000);
		// initial battery level
		currBattery = Battery.getVoltage();
	}

	// initialize
	// will do all the things needed to become part of the system
	// and get to the parked state
	// 		initialize the buttons for a controller
	//		set up communications with the controller
	// 		get the configuration parameters
	// 		perform calibration
	//      attach to the orbiter
	private void initialize() {

		buttonInit();
		Delay.msDelay(delay*1000);
		
		// establish connection to controller
		connectInit();
		Delay.msDelay(delay*1000);
		
		// get the configuration details from the controller
		configInit();	
		Delay.msDelay(delay*1000);
		
		// now we got to do stuff
		calibrate();
		Delay.msDelay(delay*1000);
		
		// attach
		// using the configuration, let out enough to attach to orbiter
		attach();
		Delay.msDelay(delay*1000);
	}
	
	// buttonInit
	private static void buttonInit() {
		if(mountPointState != MPST.MPPREINIT) {
			problem("ButtonInit");
		}
		else {
			// update our current state
			mountPointState = MPST.MPINIT;	
			LCD.clear();
			LCD.drawString("Button Init", 0, 2);
			
			// start adding listeners
			Button.ESCAPE.addButtonListener(new ButtonListener() {
				public void buttonPressed(Button b) {
					// no matter what, exit whenever ESCAPE is pressed
					LCD.clear();
					LCD.drawString(" Escape Pressed ", 0, 0);
					Delay.msDelay(1000);
					System.exit(0);
			    }
	
			    public void buttonReleased(Button b) {
			    }
			    });
			
			Button.ENTER.addButtonListener(new ButtonListener() {
				public void buttonPressed(Button b) {
					// indicate line length is set during calibration
					if (mountPointState == MPST.MPCALG) {
						mountPointState = MPST.MPCAL;
			    	}
					// indicate orbiter is attached during attaching
					if (mountPointState == MPST.MPATTG && readyToAttach == false) {
						readyToAttach = true;
					}
					else if (mountPointState == MPST.MPATTG && readyToAttach == true) {
						mountPointState = MPST.MPPARKED;
			    	}				
			    }
	
			    public void buttonReleased(Button b) {
			    }
			    });
			
			Button.LEFT.addButtonListener(new ButtonListener() {
				public void buttonPressed(Button b) {
					// if calibrating, bring in line
					if (mountPointState == MPST.MPCALG) {
						// set the speed
						myMotor.setSpeed(defaultSpeed);
						
						// set the motor in motion
						myMotor.backward();
			    	}
			    }
	
			    public void buttonReleased(Button b) {
					// if calibrating, stop the motor
					if (mountPointState == MPST.MPCALG) {
						myMotor.stop();
						LCD.drawString("tacho: "+myMotor.getTachoCount(), 0, 3);	
			    	}
			    }
			    });
			
			Button.RIGHT.addButtonListener(new ButtonListener() {
				public void buttonPressed(Button b) {
					// if calibrating, pay out line
					if (mountPointState == MPST.MPCALG) {
						// set the speed
						myMotor.setSpeed(defaultSpeed);
						
						// set the motor in motion
						myMotor.forward();

			    	}
			    }
	
			    public void buttonReleased(Button b) {
					// if calibrating, stop the motor
					if (mountPointState == MPST.MPCALG) {
						myMotor.stop();
						LCD.drawString("tacho: "+myMotor.getTachoCount(), 0, 3);	
			    	}
			    }
			    });

			LCD.drawString("Complete", 0, 3);
		}
	}
	
	// connectInit
	// the mount point has to wait for the controller to contact it
	// and the push the configuration parameters
	private static void connectInit() {
		if(mountPointState != MPST.MPINIT) {
			problem("connect");
		}
		else {
			mountPointState = MPST.MPCONNG;
			LCD.clear();
			LCD.drawString("Connect Init ", 0, 2);
			
			
			// make a connection to the controller
			ctrlConn = Bluetooth.waitForConnection();
			
			if(ctrlConn == null) {
				problem("connect.ctrlConn");
			}
			
			// set up the input / output streams
			ctrlInput = ctrlConn.openDataInputStream();
			ctrlOutput = ctrlConn.openDataOutputStream();	
			
			LCD.drawString("Complete", 0, 3);		

			// update the state and let the controller know 
			mountPointState = MPST.MPCONN;
			sendState();
			// check the battery
			checkBattery();
			// update the UI
			LCD.drawString("Complete", 0, 3);	
		}		
	}	
	
	// configInit
	// receive all the basic constant values that apply to this setup
	// requires that connection has already been established
	private static void configInit() {
		
		if (mountPointState != MPST.MPCONN) {
			problem("initialize");
		}
		else {
			// update the state
			mountPointState = MPST.MPCONFG;
			sendState();
			
			// update the display
			LCD.clear();
			LCD.drawString("Config Init", 0, 2);
			
			// receive input from controller and set parameters	
			try { 
				heightAboveGround = ctrlInput.readFloat(); 
				mountPointSpacing = ctrlInput.readFloat();
				orbitRadius		  = ctrlInput.readFloat();
				orbiterDipSpacing = ctrlInput.readFloat();
				startStopTime	  = ctrlInput.readInt();
				timeToOrbit 	  = ctrlInput.readInt();
				
				// acknowledge receipt
				LCD.drawString("HAG "+heightAboveGround+";MPS "+mountPointSpacing, 0, 2);	
				LCD.drawString("ORB "+orbitRadius+";ODS "+orbiterDipSpacing, 0, 3);	
				LCD.drawString("SST "+startStopTime+";TTO "+timeToOrbit, 0, 4);	

				// may as well calculate our lengths and stuff . . . 
				// need to adjust for the height and width of attachment point
				parkedLen	= (float) Math.sqrt(((heightAboveGround-orbiterHeight)*(heightAboveGround-orbiterHeight)) + (((mountPointSpacing/Math.sqrt(2.0))-orbiterWidth)*((mountPointSpacing/Math.sqrt(2.0))-orbiterWidth)));
				// no adjustment in height for this one - we will consider the dip to be from the top of the mount point to the top of the orbiter
				stoppedLen 	= (float) Math.sqrt((orbiterDipSpacing*orbiterDipSpacing) + (((mountPointSpacing/Math.sqrt(2.0))-orbiterWidth)*((mountPointSpacing/Math.sqrt(2.0))-orbiterWidth)));
				
				LCD.drawString("PL: "+Math.round(parkedLen)+" SL: "+Math.round(stoppedLen), 0, 5);
				
				// calculate mountDistance and timeOffset
				mountDistance = (mountPointSpacing / Math.sqrt(2.0)) - orbitRadius;
				timeOffset = (timeToOrbit / 4.0F) * (float) (whoAmI - 1);
				LCD.drawString("MD: "+Math.round(mountDistance)+" TO: "+timeOffset, 0, 6);
				
				Delay.msDelay(5000);
				
				// update the state and notify controller
				mountPointState = MPST.MPCONF;
				sendState();	
				// check the battery
				checkBattery();
				// update the UI
				LCD.drawString("Complete", 0, 7);

			}
			catch(Exception e) {
				e.printStackTrace();
			}
		}
	}
	
	// calibrate
	// lower the line to the floor
	private static void calibrate() {
		if(mountPointState != MPST.MPCONF) {
			problem("calibrate");
		}
		else {
			// set our state
			mountPointState = MPST.MPCALG;
			// communicate our state
			sendState();
			// update the UI
			LCD.clear();
			LCD.drawString("Calibrating", 0, 2);
			
			// do calibration
			// right button lets out line, left button takes up line, enter indicates calibration is complete
			// state was updated by buttons
			while(mountPointState != MPST.MPCAL) {
				// keep going until the button is released
				LCD.drawString("tacho: "+myMotor.getTachoCount(), 0, 3);	
				Delay.msDelay(500);
			}
			
			// notify controller
			sendState();	
			// check the battery
			checkBattery();
			// update the UI
			LCD.drawString("Complete", 0, 4);

		}
	}
		
	// attach
	// after calibration, connect the mountpoint to the orbiter
	private static void attach() {
		if (mountPointState != MPST.MPCAL) {
			problem("attach");
		}
		else {
			readyToAttach = false;
			mountPointState = MPST.MPATTG;
			sendState();
			LCD.clear();
			LCD.drawString("Attaching", 0, 2);
			
			// use input from controller and set parameters	
			// let out enough line to attach to orbiter
			// need a total of SQRT(HAG^2 + MPS^2/2)
			// already have extended HAG, so subtract that
			// two-step process
			// 1. hit enter when ready to attach this mount point
			// 2. hit enter again when attached
			LCD.drawString("ROT:"+(int) Math.round((360.0F*(parkedLen - heightAboveGround)/(2.0F*Math.PI*spoolRadius*gearRatio))), 0, 3);
			LCD.drawString("waiting . . . ",0,4);
			while(! readyToAttach) {
				Delay.msDelay(100);
			}
			myMotor.rotate((int) Math.round((360.0F*(parkedLen - heightAboveGround)/(2.0F*Math.PI*spoolRadius*gearRatio))));
			
			// need some time to attach - let the button handle the state change
			while(mountPointState != MPST.MPPARKED) {
				Delay.msDelay(100);
			}
			myMotor.stop();
			
			// notify controller
			sendState();	
			// check the battery
			checkBattery();
			// update the UI
			LCD.drawString("Complete        ", 0, 5);	
		}		
	}

	private static void checkBattery() {
		MPST newBatteryState;

		// supposed accurate maximum is 110 * battery voltage
		currBattery = Battery.getVoltage();

		if(currBattery / maxBattery >= 0.9F) {
			newBatteryState = MPST.MPBATT90;
		}
		else if(currBattery / maxBattery >= 0.75F) {
			newBatteryState = MPST.MPBATT75;
		}
		else if(currBattery / maxBattery >= 0.5F) {
			newBatteryState = MPST.MPBATT50;
		}
		else if(currBattery / maxBattery >= 0.25F) {
			newBatteryState = MPST.MPBATT25;
		}
		else newBatteryState = MPST.MPBATT0;
			
		if(batteryState != newBatteryState) {
			batteryState = newBatteryState;
			// sendstate somehow
		}
		
	}
	
	// send the communication of state to the controller
	private static void sendState() {
		try {
			LCD.drawString("send: "+mountPointState.toString(), 0, 7);
			LCD.refresh();
			Delay.msDelay(1000);
			ctrlOutput.writeUTF(mountPointState.toString());
			ctrlOutput.flush();
		}
		catch(Exception e) {
			e.printStackTrace();
		}
	}
	
	// stateMonitor
	// keep an eye on messages from controller and change state accordingly
	// this should block and only run when a message from the controller is received
	private class ControllerStateMonitor extends Thread {
		private DataInputStream dis;
		private CTRLST			newState;
		
		public ControllerStateMonitor(DataInputStream dis) {
			this.dis = dis;
		}
		
		// read input from controller and update state accordingly
		public void run() {
			// enum.valueOf does not work
			// read string and use switch to get enum value
			String stateName;
			
			// just keep looping until told to do otherwise		
			// will block on read, so not so inefficient
			while(true) {
				try {
					stateName = dis.readUTF();
					LCD.drawString(stateName, 0, 7);
					LCD.refresh();
					switch(stateName) {
						case "CTRLINIT"	:	newState = CTRLST.CTRLINIT; break;
						case "PARKED"	:	newState = CTRLST.PARKED; break;
						case "RAISING"	:	newState = CTRLST.RAISING; break;
						case "STOPPED"	:	newState = CTRLST.STOPPED; break;
						case "STARTING"	:	newState = CTRLST.STARTING; break;
						case "ORBITING"	:	newState = CTRLST.ORBITING; break;
						case "STOPPING"	:	newState = CTRLST.STOPPING; break;
						case "LOWERING"	:	newState = CTRLST.LOWERING; break;
						case "HALT"		:	newState = CTRLST.HALT; break;
						default: 
					}

					if(newState != orbiterState) {
						orbiterState = newState;
					}
				}
				catch (IOException ioe) {
					LCD.drawString("Read Exception ", 0, 0);
					LCD.refresh();
					Delay.msDelay(10000);
					System.exit(1);
				}	
				catch (Exception e) {
					LCD.drawString("got an error", 0, 1);
					LCD.refresh();
				}
			}
		}
	}
	
	private class StartStopManager extends Thread {
		private int pollingInterval = 10;	// amount of time in milliseconds between updates of currentRadius
		private MPST mpst;

		private StartStopManager(MPST mpst) {
			this.mpst = mpst;
		}
		
		public void run() {
			// we are going to loop for a specific amount of time to either increase or decrease 
			// the radius for the orbiter, depending if we are starting or stopping
			// the thread only exists for the length of time required to start or stop
			
			if(mpst == MPST.MPSTARTING) {
				// expand
				LCD.drawString("sst:            ", 0, 5);
				LCD.drawString("sst: "+startStopTime, 0, 5);
				
				for(int i = 0; i <= (1000*startStopTime/pollingInterval); i++) {
					currentRadius = orbitRadius * (((float)(i * pollingInterval))/(startStopTime*1000.0F));
					LCD.drawString("start:          ", 0, 6);
					LCD.drawString("start: "+(int)currentRadius, 0, 6);
					LCD.refresh();
					Delay.msDelay(pollingInterval);
				}
				// done! set state
				endStarting();				
			}
			else if (mpst == MPST.MPSTOPPING) {
				// contract
				LCD.drawString("sst:            ", 0, 5);
				LCD.drawString("sst: "+startStopTime, 0, 5);
				for(int i = 0; i <= (1000*startStopTime/pollingInterval); i++) {
					currentRadius = orbitRadius - (orbitRadius * ((float)(i * pollingInterval)/(startStopTime*1000.0F)));
					LCD.drawString("stop:           ", 0, 6);
					LCD.drawString("stop: "+(int)currentRadius, 0, 6);
					LCD.refresh();
					Delay.msDelay(pollingInterval);
				}
				// done! set state and stop motor
				endStopping();
			}
			
			// thread is done - exit normally
			LCD.drawString("startstop done", 0, 6);
		}
	}
	
	// this class does all the work
	private class OrbitManager extends Thread {
		long	baseTime;		// initial time reading
//		int		currTach;		// most recent tach reading
		int		prevTach;		// prior tach reading
		double  calcTach;		// what we think the tach should read
		int		currSpeed;		// most recent speed reading
		int		nextSpeed;
		long	currTime;		// most recent time reading
		long	prevTime;		// prior time reading
		double  timeInterval;   // time between samples, in milliseconds; estimated
		int		loopInterval;   // wait time between loops
		double	currLength;	
		double	nextLength;
//		float	battVolt  	= 0;		// battery voltage - currently only pulling at start
		boolean motorForward;
		
		
		private double getLineLength(float timeNow) {
			// timeNow is in milliseconds; timeToOrbit is in seconds
			// include the adjustment depending on what mount point we are 
			/* -- original
			double  theta = 360*((timeNow + timeOffset) % (1000*timeToOrbit))/(1000*timeToOrbit);
			double  beta  = (mountDistance + orbitRadius - (orbitRadius * Math.cos(Math.toRadians(theta))));
			return Math.sqrt((orbiterDipSpacing*orbiterDipSpacing) + (beta*beta) + (orbitRadius*Math.sin(Math.toRadians(theta))));
			-- */
			// fixed so orbit radius can change when stopping / starting
			/* -- second take
			mountDistance = (mountPointSpacing / Math.sqrt(2.0)) - currentRadius;
			double  theta = 360*((timeNow + timeOffset) % (1000.0D*timeToOrbit))/(1000.0D*timeToOrbit);
			double  beta  = (mountDistance + currentRadius - (currentRadius * Math.cos(Math.toRadians(theta))));
			return Math.sqrt((orbiterDipSpacing*orbiterDipSpacing) + (beta*beta) + (currentRadius*Math.sin(Math.toRadians(theta))));
			-- */
			// third attempt
			mountDistance = (mountPointSpacing / Math.sqrt(2.0)) - currentRadius;
			double beta   = 2.0D * Math.PI / timeToOrbit;
			double theta  = beta * timeNow/1000.0D;				// correct seconds and milliseconds
			double plx    = currentRadius * Math.cos(theta);
		    double ply    = currentRadius * Math.sin(theta);
		    double mpx	  = (currentRadius + mountDistance) * Math.cos(beta * timeOffset);
		    double mpy    = (currentRadius + mountDistance) * Math.sin(beta * timeOffset);
		    double ll	  = Math.sqrt(((mpx-plx)*(mpx-plx))+((mpy-ply)*(mpy-ply)));
		    return Math.sqrt((ll*ll)+(orbiterDipSpacing*orbiterDipSpacing));			
		}
		
		public OrbitManager() {
			initialize();
		}
		
		private void initialize() {
			baseTime  	= 0L;		// initial time reading
//			currTach  	= 0;		// most recent tach reading
//			prevTach  	= 0;		// prior tach reading
			calcTach  	= 0D;		// what we think the tach should read
			currSpeed 	= 0;		// most recent speed reading
			nextSpeed 	= 0;
			currTime  	= 0L;		// most recent time reading
			prevTime  	= 0L;		// prior time reading
			currLength 	= 0D;	
			nextLength 	= 0D;
			timeInterval = 30D;
			loopInterval = 5;
			motorForward = true;
		}
		
		// will do one cycle of starting and stopping, then exit
		public void run() {
			// set the speed
			// value is in degrees per second
			myMotor.setSpeed(0);
			
			// set the motor in motion
			myMotor.forward();

			baseTime = System.currentTimeMillis();	// initialize so that everything is relative to our start time
			
			while (mountPointState == MPST.MPSTARTING || mountPointState == MPST.MPORBITING || mountPointState == MPST.MPSTOPPING) {
				// get the next length
				currTime = System.currentTimeMillis();
				currLength = getLineLength(currTime-baseTime);
				nextLength = getLineLength((float) (currTime-baseTime + timeInterval));
				
				// get the difference between the current length and the next length
				nextLength = nextLength - currLength;

//				// calculate expected tach 
//				prevTach = currTach;
//				currTach = myMotor.getTachoCount();
//				prevTime = currTime;
				currSpeed = myMotor.getSpeed();

				// calculate expected tach 
//				calcTach = motorForward ? calcTach + (currSpeed * ((currTime - prevTime)/1000.0F)) : calcTach - (currSpeed * ((currTime - prevTime)/1000.0F));
						
				// calculate the speed to make the difference
				// motor speed is in degrees per second
				// we need to calculate that based on how far the spool should move during the time interval
				// incorporate gear ratio
				// account for time (needs to convert to seconds)
//				nextSpeed = (int) (Math.toDegrees(nextLength / spoolRadius) * (1.0F/gearRatio) * (1000.0F/30.0F)) ;
				nextSpeed = (int) (360.0F*((nextLength / (2.0F*Math.PI*spoolRadius)) * (1.0F/gearRatio) * (1000.0F/timeInterval))) ;

				// status
				LCD.drawString("c speed:        ", 0, 2);
				LCD.drawString("c speed: "+currSpeed, 0, 2);
				LCD.drawString("n speed:        ", 0, 3);
				LCD.drawString("n speed: "+nextSpeed, 0, 3);
				LCD.drawString("len:            ", 0, 4);
				LCD.drawString("len: "+nextLength, 0, 4);
				LCD.drawString("dir:            ", 0, 5);
				LCD.drawString("dir: "+ (motorForward ? "forward" : "backward"), 0, 5);
				LCD.refresh();
				
				// change the motor speed
				myMotor.setSpeed(Math.abs(nextSpeed));
				if(nextSpeed>0) {
					motorForward = true;
					myMotor.forward();
				}
				else	{
					motorForward = false;
					myMotor.backward();
				}
				
//				// get info and print it out
//				RConsole.println("time: "+(currTime-baseTime) + "; curr speed: "+currSpeed+"; next speed: "+ nextSpeed +
//						 "; curr tach: "+currTach+"; calcTach: "+calcTach+
//						 "; time interval: "+(currTime - prevTime)+"; currLength: "+currLength+"; nextLength: "+nextLength);
				
				// wait for next loop
				Delay.msDelay(loopInterval);								
			}
		}
	}

	
	// problem
	// quick error handler 
	private static void problem(String problem) {
		LCD.clear();
		LCD.drawString(problem, 0, 3);
		Sound.twoBeeps();
		Delay.msDelay(1000);
		System.exit(0);
	}
	
	private void beginRaising() {
		if(mountPointState != MPST.MPPARKED) {
			problem("beginRaising");
		}
		else {
			// update the state
			mountPointState = MPST.MPRAISING;
			sendState();
			
			// update the display
			LCD.clear(); 
			LCD.drawString("RAISING", 0, 0);
			LCD.drawString("ROT: "+(int) Math.round((360.0F*(stoppedLen - parkedLen)/(2.0F*Math.PI*spoolRadius*gearRatio))), 0, 1);
			LCD.drawString("SLL: "+stoppedLen, 0, 2);
			LCD.refresh();
			
			// make it happen
			// already have extended parkedLen, so subtract that
			// stoppedLen should be shorter than parkedLen, so I think motor will rotate backwards
			myMotor.setSpeed(defaultSpeed);
			myMotor.rotate((int) Math.round((360.0F*(stoppedLen - parkedLen)/(2.0F*Math.PI*spoolRadius*gearRatio))),true);
		}
	}

	private void endRaising() {
		// update the state 
		mountPointState = MPST.MPSTOPPED;
		sendState();
		
		// update the display
		LCD.clear(); 
		LCD.drawString("STOPPED", 0, 0);
	}
	
	private void beginStopping() {
		if(mountPointState != MPST.MPORBITING) {
			problem("beginStopping");
		}
		else {
			// update the state
			mountPointState = MPST.MPSTOPPING;
			sendState();
			
			// update the display
			LCD.clear(); 
			LCD.drawString("STOPPING", 0, 0);
			
			// make it happen
			ss = new StartStopManager(mountPointState);
			ss.start();
		}
	}

	private void endStopping() {	
		// in this case, we will stop the motor
		// it is probably not moving, but we are still sending commands
		myMotor.stop();
		
		// update the state 
		mountPointState = MPST.MPSTOPPED;
		sendState();
				
		// update the display
		LCD.clear(); 
		LCD.drawString("STOPPED", 0, 0);
	}

		
	private void beginLowering() {
		if(mountPointState != MPST.MPSTOPPED) {
			problem("beginLowering");
		}
		else {
			// update the state
			mountPointState = MPST.MPLOWER;
			sendState();
			
			// update the display
			LCD.clear(); 
			LCD.drawString("LOWERING", 0, 0);
			LCD.drawString("ROT:"+(int) Math.round((360.0F*(parkedLen - stoppedLen)/(2.0F*Math.PI*spoolRadius*gearRatio))), 0, 1);
			LCD.drawString("PLL: "+parkedLen, 0, 2);
			LCD.refresh();
			
			// make it happen
			// already have extended stoppedLen, so subtract that
//			LCD.drawString("I am here", 0, 3);
			myMotor.setSpeed(defaultSpeed);
			myMotor.rotate((int) Math.round((360.0F*(parkedLen - stoppedLen))/((2.0F*Math.PI*spoolRadius*gearRatio))), true);
//			LCD.drawString("Now I am not", 0, 4);
//			Delay.msDelay(3000);
			
		}
	}
	
	private void endLowering() {
		// update the state again
		mountPointState = MPST.MPPARKED;
		sendState();
		
		// update the display
		LCD.clear(); 
		LCD.drawString("PARKED", 0, 0);
		LCD.refresh();
	}

	private void beginStarting() {
		if(mountPointState != MPST.MPSTOPPED) {
			problem("beginStarting");
		}
		else {
			// update the state
			mountPointState = MPST.MPSTARTING;
			sendState();
			
			// update the display
			LCD.clear(); 
			LCD.drawString("STARTING", 0, 0);
			LCD.refresh();

			// make it happen
			// create the thread objects - need one of each
			orbMgr = new OrbitManager();
			ss = new StartStopManager(mountPointState);
			orbMgr.start();
			ss.start();
		}
	}
	
	private void endStarting() {
		// update the state 
		mountPointState = MPST.MPORBITING;
		sendState();
		
		// update the display
		LCD.clear(); 
		LCD.drawString("ORBITING", 0, 0);
		LCD.refresh();
	}
	
	private void doHalt() {
		// quit immediately
		myMotor.stop();
		
		// update the state
		mountPointState = MPST.MPHALT;
		sendState();
		
		// update the display
		LCD.clear(); 
		LCD.drawString("HALTED", 0, 0);
		LCD.refresh();
	}
	
	// run - main thread
	// override of standard thread run call
	public void run() {
		
		// temp variable 
		CTRLST currentState = orbiterState;
			
		// set up controller listener
		// it will catch messages sent by controller
		gcs = new ControllerStateMonitor(ctrlInput);
		gcs.start();
		
		// for now, loop until we get a halt from the controller
		// orbiterState is set by getCtrlState thread
		while(orbiterState != CTRLST.HALT) {
			// check mount point state first
			// we care if we are lowering, raising, starting, or stopping
			// because motor will be set in motion and we need to see if it has finished to change state
			if(! myMotor.isMoving()) {
				switch(mountPointState) {
					case MPRAISING: 	endRaising(); 	break;
					case MPLOWER: 		endLowering(); 	break; 
					case MPSTARTING: 	endStarting();	break;   // can really ignore this . . . taken care of elsewhere
					case MPSTOPPING:	endStopping();	break;
					default: break;					// do nothing otherwise
				}
			}
			if(currentState != orbiterState) {
				// current state of the orbiter has changed - do something
				currentState = orbiterState;
				switch (orbiterState) {
					case CTRLINIT:		LCD.clear(); LCD.drawString("INITIALIZE", 0, 0); break;
					case RAISING: 		beginRaising();  break;
					case STARTING: 		beginStarting(); break;
					case STOPPING: 		beginStopping(); break;
					case LOWERING: 		beginLowering(); break;
					case HALT: 			doHalt(); 		 break;
					case PARKED: 		// ignore
					case ORBITING: 		// ignore
					case STOPPED: 		// ignore 
										break;
					default: 			LCD.clear(); LCD.drawString("beats me", 0, 0); break;
				}
			}
		}
		// halt received - finish and clean up

		Sound.beepSequence();
		LCD.clear();
		LCD.drawString("HALTED.",0,0);
		LCD.refresh();
		Delay.msDelay(10000);
		
	}
	
	public static void main(String[] args) {
		MountPoint2 mp2 = new MountPoint2();
		
		// do the setup to get to the parked state
		mp2.initialize();
		
		// ready to rock
		mp2.start();
		
	}
}
