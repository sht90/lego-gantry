package com.stan;

import java.util.ArrayList;
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

	// this class originally had the docstring "this class does all the work"
	// and yea that's about right. The orbiter manager needs to think about:
	// - motor position
	// - motor speed
	// - time
	// - progress along the spline
	private class OrbitManager extends Thread {	// time stuff
		long	baseTime;		// initial time reading
		long	currTime;		// most recent time reading
		long	prevTime;		// prior time reading
		double  timeInterval;   // time between samples, in milliseconds; estimated
		int		loopInterval;   // wait time between loops
		// motor stuff
		//int	currTach;		// most recent tach reading
		int		prevTach;		// prior tach reading
		double  calcTach;		// what we think the tach should read
		int		currSpeed;		// most recent speed reading
		int		nextSpeed;
		boolean motorForward;
		//float battVolt  	= 0;		// battery voltage - currently only pulling at start
		// stuff about the line, especially its length
		double	currLength;	
		double	nextLength;
		// !! NEW !!
		int mountPointIndex;
		double[] mountPoint;
		double[][] allMountPoints;
		//double	mountPointX;
		//double	mountPointY;
		//double	mountPointZ;
		double[] endEffectorPosition;
		//double	endEffectorX;
		//double	endEffectorY;
		//double	endEffectorZ;
		double[][] controlPoints;
		double[] speeds;

		// --- PURE MATH STUFF ---

		// calculate 3D Euclidean distance
		public double distance(double[] xyz1, double[] xyz2) {
			double dx = xyz2[0] - xyz1[0];
			double dy = xyz2[1] - xyz1[1];
			double dz = xyz2[2] - xyz1[2];
			return Math.sqrt(dx * dx + dy * dy + dz * dz);
		}

		// linearly interpolate between a and b, where p is between 0 and 1 inclusive
		public double lerp1d(double a, double b, double p) {
			return (1 - p) * a + p * b;
		}

		// lerp n-dimensional vectors v1 and v2, where p is between 0 and 1 inclusive
		// v1.length == v2.length must be true
		public double[] lerpnd(double[] v1, double[] v2, double p) {
			if (v1.length != v2.length) {
				return null;
			}
			double[] retval = new double[v1.length];
			for (int i = 0; i < retval.length; i++) {
				retval[i] = lerp1d(v1[i], v2[i], p);
			}
			return retval;
		}

		// matrix multiplication, I think ripped from here https://medium.com/@AlexanderObregon/matrix-multiplication-logic-in-java-with-strassens-method-0f8e9feec298
		// though I probably could've ripped it from wikipedia just as easily https://en.wikipedia.org/wiki/Matrix_multiplication_algorithm
		public double[][] matmul(double[][] a, double[][] b) {
			int m = a.length;
			int n = a[0].length;
			if (n != b.length) {
				throw new IllegalArgumentException("Incompatible matrices bad");
			}
			int p = b[0].length;

			double[][] c = new double[m][p];
			for (int i = 0; i < m; i++) {
				for (int j = 0; j < p; j++) {
					double sum = 0;
					for (int k = 0; k < n; k++) {
						sum += a[i][k] * b[k][j];
					}
					c[i][j] = sum;
				}
			}
			return c;
		}

		// get the product of a scalar and a vector
		public double[] vectorScalarMult(double s, double[] v) {
			double[] vmult = new double[v.length];
			for (int i = 0; i < v.length; i++) {
				vmult[i] = v[i] * s;
			}
			return vmult;
		}

		// take the dot product of two vectors
		public double vectorDot(double[] v1, double[] v2) {
			double vdot = 0.0;
			for (int i = 0; i < v1.length; i++) {
				vdot = vdot + v1[i] * v2[i];
			}
			return vdot;
		}

		// add two vectors together
		public double[] vectorSum(double[] v1, double[] v2) { 
			double[] vsum = new double[v1.length];
			for (int i = 0; i < v1.length; i++) {
				vsum[i] = v1[i] + v2[i];
			}
			return vsum;
		}

		// subtract vector v2 from vector v1
		public double[] vectorDifference(double[] v1, double[] v2) {
			double[] vdiff = new double[v1.length];
			for (int i = 0; i < v1.length; i++) {
				vdiff[i] = v1[i] - v2[i];
			}
			return vdiff;
		}

		// calculate the scalar magnitude of a vector
		public double vectorMagnitude(double[] v) {
			double magnitudeSquared = 0;
			for (int i = 0; i < v.length; i++) {
				magnitudeSquared = magnitudeSquared + v[i] * v[i];
			}
			return Math.sqrt(magnitudeSquared);
		}

		// normalize a vector, i.e. get a unit vector with the same direction as the input vector
		public double[] normalizeVector(double[] v) {
			double magnitude = vectorMagnitude(v);
			double[] normalizedVector = new double[v.length];
			for (int i = 0; i < v.length; i++) {
				normalizedVector[i] = v[i] / magnitude;
			}
			return normalizedVector;
		}

		// Generate points on a b-spline
		// This doesn't automatically clamp to the control points,
		// but you can achieve that same effecting by copying the first and last control points 3 times.
		public double[][] generatePointsOnBSpline(int subsampling, double[][] controlPoints) {
			ArrayList<double[]> splineXYZ = new ArrayList<double[]>();
			double[] t = new double[controlPoints.length * subsampling + 1];
			for (int i = 0; i < t.length; i++) {
				t[i] = lerp1d(0.0, controlPoints.length, i * 1.0 / (t.length - 1));
			}
			double[][] characteristic_matrix = {
				{ 1.0/6, 4.0/6, 1.0/6, 0.0/6},
				{-3.0/6, 0.0/6, 3.0/6, 0.0/6},
				{ 3.0/6,-6.0/6, 3.0/6, 0.0/6},
				{-1.0/6, 3.0/6,-3.0/6, 1.0/6}};
			for (int i = 0; i < controlPoints.length; i++) {
				if (i == 0 || i >= controlPoints.length - 2) {
					continue;
				}
				for (int j = subsampling * i; j < subsampling * (i + 1); j++) {
					double u = t[j] % 1;
					double[][] polynomialTerms = {{1, u, u * u, u * u * u}};
					double[][] points = new double[4][3];
					for (int k = i - 1; k < i + 3; k++) {
						for (int l = 0; l < 3; l++) {
							points[k - (i - 1)][l] = controlPoints[k][l];
						}
					}
					double[][] curveMat = matmul(polynomialTerms, matmul(characteristic_matrix, points));
					double[] curvePoint = new double[curveMat[0].length];
					for (int k = 0; k < curvePoint.length; k++) {
						curvePoint[k] = curveMat[0][k];
					}
					splineXYZ.add(curvePoint);
				}
			}
			double[][] splineXYZarray = new double[splineXYZ.size()][];
			for (int i = 0; i < splineXYZ.size(); i++) {
				splineXYZarray[i] = splineXYZ.get(i);
			}
			return splineXYZarray;
		}

		// Generate velocities on a b-spline, clamping to end control points
		public double[][] generateVelocitiesOnBSpline(int subsampling, double[][] controlPoints) {
			ArrayList<double[]> splineXYZ = new ArrayList<double[]>();
			double[] t = new double[controlPoints.length * subsampling + 1];
			for (int i = 0; i < t.length; i++) {
				t[i] = lerp1d(0.0, controlPoints.length, i * 1.0 / (t.length - 1));
			}
			double[][] characteristic_matrix = {
				{ 1.0/6, 4.0/6, 1.0/6, 0.0/6},
				{-3.0/6, 0.0/6, 3.0/6, 0.0/6},
				{ 3.0/6,-6.0/6, 3.0/6, 0.0/6},
				{-1.0/6, 3.0/6,-3.0/6, 1.0/6}};
			for (int i = 0; i < controlPoints.length; i++) {
				if (i == 0 || i >= controlPoints.length - 2) {
					continue;
				}
				for (int j = subsampling * i; j < subsampling * (i + 1); j++) {
					double u = t[j] % 1;
					double[][] polynomialTerms = {{0, 1, 2 * u, 3 * u * u}};
					double[][] points = new double[4][3];
					for (int k = i - 1; k < i + 3; k++) {
						for (int l = 0; l < 3; l++) {
							points[k - (i - 1)][l] = controlPoints[k][l];
						}
					}
					double[][] curveMat = matmul(polynomialTerms, matmul(characteristic_matrix, points));
					double[] curvePoint = new double[curveMat[0].length];
					for (int k = 0; k < curvePoint.length; k++) {
						curvePoint[k] = curveMat[0][k];
					}
					splineXYZ.add(curvePoint);
				}
			}
			double[][] splineXYZarray = new double[splineXYZ.size()][];
			for (int i = 0; i < splineXYZ.size(); i++) {
				splineXYZarray[i] = splineXYZ.get(i);
			}
			return splineXYZarray;
		}

		// calculate wire length between a mount point and the end effector
		public double[] generateWireLengths(double[][] pt, double[] towerPosition) {
			double[] wireLengths = new double[pt.length];
			for (int i = 0; i < pt.length; i++) {
				wireLengths[i] = distance(pt[i], towerPosition);
			}
			return wireLengths;
		}

		// calculate wire velocity, where + means letting out wire and - means reeling in wire
		public double[] generateWireVelocity(double[][] vt, double[][] pt, double[] towerPosition) {
			double[] wireVelocities = new double[vt.length];
			for (int i = 0; i < vt.length; i++) {
				double[] endEffectorPosition = pt[i];
				double[] endEffectorVelocity = vt[i];
				double[] wireVector = vectorDifference(endEffectorPosition, towerPosition);
				double[] wireDirection = normalizeVector(wireVector);
				double[] velocityAlongWire3d = vectorScalarMult(vectorDot(endEffectorVelocity, wireDirection), wireDirection);
				double speedAlongWire = vectorMagnitude(velocityAlongWire3d);
				double directionAlongWire = 1;
				if (vectorDot(velocityAlongWire3d, wireDirection) > 0) {
					directionAlongWire = -1;
				}
				double velocityAlongWire1d = speedAlongWire * directionAlongWire;
				wireVelocities[i] = velocityAlongWire1d;
			}
			return wireVelocities;
		}

		// --- END MATH STUFF ---

		// --- ORBITER SPECIFIC STUFF ---

		// wrapper for myMotor.forward() so I don't get confused about what + or forward means
		// if you let out line at a negative speed, you will reel in line
		private void letOutLine() {
			myMotor.forward();
		}

		// wrapper for myMotor.backward() so I don't get confused about what - or backward means
		// if you reel in line at a negative speed, you will let out in line
		private void reelInLine() {
			myMotor.backward();
		}

		// TODO: I think this may be unnecessary, I might only need goal speeds, not goal positions?
		// get the goal position of the end effector for a given time
		private double[] getEndEffectorGoalPosition(float timeNow){
			// assume a constant time interval between control points
			// assume a uniform time distribution between any two control points
			// TODO: actually get an end effector goal position from time
			return null;
		}

		// TODO: I think this may be unnecessary, I might only need goal speeds, not goal positions?
		// get the length of the line, given the time
		private double getLineLength(float timeNow) {
			// Assuming that the line is approximately straight, (which, we are making that assumption),
			// the only hard part of this function is knowing where the end effector is.
			endEffectorPosition = getEndEffectorGoalPosition(timeNow);
			return distance(endEffectorPosition, mountPoint);
		}

		// get the motor speed, as an int number of degrees per second,
		// from lineSpeed and the transmission between the motor and the line (spool and gears)
		private int getMotorSpeedFromLineSpeed(double lineSpeed, double spoolRadius, double gearRatio) {
			return (int)(360.0F * lineSpeed / (2.0F * Math.PI * spoolRadius) / gearRatio);
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
			//timeInterval = 30D;  // I define this later based on other stuff.
			// fwiw I'd rather derive subsampling interval from time interval instead of the other way around, 
			// but math is hard
			loopInterval = 5;
			motorForward = true;
			// !! NEW !!
			allMountPoints = new double[][] {
				{ 0D,  0D, 40D},
				{40D,  0D, 40D},
				{ 0D, 40D, 40D},
				{40D, 40D, 40D}
			};
			mountPointIndex = 1;
			mountPoint = allMountPoints[mountPointIndex];
			endEffectorPosition = new double[] {0D, 0D, 0D};
			// remember to triplicate the first and last points. B-Splines don't necessarily pass through unique control points on their own.
			controlPoints = new double[][] {
				{0D, 0D, 0D},
				{0D, 0D, 0D},
				{0D, 0D, 0D},
				{0D, 0D, 10D},
				{0D, 0D, 10D},
				{0D, 0D, 10D}
			};
			// Parse the control points into positions and velocities
			int subsampling = 10;
			double maxAllowedMotorSpeed = 700.0; // deg/s
			double[][] splinePoints = generatePointsOnBSpline(subsampling, controlPoints);
			// Get wire lengths for each point
			double[][] allWireLengths = new double[allMountPoints.length][];
			for (int i = 0; i < allMountPoints.length; i++) {
				allWireLengths[i] = generateWireLengths(splinePoints, allMountPoints[i]);
			}
			// Find max distance covered by any spool over any time interval
			double maxDistanceCovered = 0;
			double distanceCovered = 0;
			for (int i = 0; i < allWireLengths.length; i++) {
				for (int j = 1; j < allWireLengths[i].length; j++) {
					distanceCovered = Math.abs(allWireLengths[i][j] - allWireLengths[i][j - 1]);
					if (distanceCovered > maxDistanceCovered) {
						maxDistanceCovered = distanceCovered;
					}
				}
			}
			// Impose the highest possible velocity on the maximum distance covered
			double speedMultiplier = maxAllowedMotorSpeed / maxDistanceCovered;
			double[][] allWireVelocities = new double[allWireLengths.length][allWireLengths[0].length];
			for (int i = 0; i < allWireLengths.length; i++) {
				for (int j = 0; j < allWireLengths[i].length; j++) {
					allWireVelocities[i][j] = allWireLengths[i][j] * speedMultiplier;
				}
			}
			// and then we know that the time interval between each point in the spline is
			// maxDistanceCovered (inches) * transmission? (deg / inch) / maxAllowedMotorSpeed (deg/s) = time (s)
			timeInterval = maxDistanceCovered * gearRatio * spoolRadius / maxAllowedMotorSpeed;
		}

		// will do one cycle of starting and stopping, then exit
		public void run() {
			myMotor.setSpeed(0);  // set the speed, in degrees per second
			//myMotor.forward();  // set the motor in motion
			baseTime = System.currentTimeMillis();	// initialize so that everything is relative to our start time
			
			// main loop:
			// - calculate goal line length based on current time
			// - calculate motor speed to achieve goal line length
			// - display line length and motor speed to screen
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
