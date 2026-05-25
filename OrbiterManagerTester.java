import java.util.ArrayList;
import java.util.Arrays;
public class OrbiterManagerTester extends Thread {
	// time stuff
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

	// --- PURE MATH STUFF ---

	// calculate 3D Euclidean distance
	public static double distance(double[] xyz1, double[] xyz2) {
		double dx = xyz2[0] - xyz1[0];
		double dy = xyz2[1] - xyz1[1];
		double dz = xyz2[2] - xyz1[2];
		return Math.sqrt(dx * dx + dy * dy + dz * dz);
	}

	// linearly interpolate between a and b, where p is between 0 and 1 inclusive
	public static double lerp1d(double a, double b, double p) {
		return (1 - p) * a + p * b;
	}

	// lerp n-dimensional vectors v1 and v2, where p is between 0 and 1 inclusive
	// v1.length == v2.length must be true
	public static double[] lerpnd(double[] v1, double[] v2, double p) {
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
	public static double[][] matmul(double[][] a, double[][] b) {
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
	public static double[] vectorScalarMult(double s, double[] v) {
		double[] vmult = new double[v.length];
		for (int i = 0; i < v.length; i++) {
			vmult[i] = v[i] * s;
		}
		return vmult;
	}

	// take the dot product of two vectors
	public static double vectorDot(double[] v1, double[] v2) {
		double vdot = 0.0;
		for (int i = 0; i < v1.length; i++) {
			vdot = vdot + v1[i] * v2[i];
		}
		return vdot;
	}

	// add two vectors together
	public static double[] vectorSum(double[] v1, double[] v2) { 
		double[] vsum = new double[v1.length];
		for (int i = 0; i < v1.length; i++) {
			vsum[i] = v1[i] + v2[i];
		}
		return vsum;
	}

	// subtract vector v2 from vector v1
	public static double[] vectorDifference(double[] v1, double[] v2) {
		double[] vdiff = new double[v1.length];
		for (int i = 0; i < v1.length; i++) {
			vdiff[i] = v1[i] - v2[i];
		}
		return vdiff;
	}

	// calculate the scalar magnitude of a vector
	public static double vectorMagnitude(double[] v) {
		double magnitudeSquared = 0;
		for (int i = 0; i < v.length; i++) {
			magnitudeSquared = magnitudeSquared + v[i] * v[i];
		}
		return Math.sqrt(magnitudeSquared);
	}

	// normalize a vector, i.e. get a unit vector with the same direction as the input vector
	public static double[] normalizeVector(double[] v) {
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
	public static double[][] generatePointsOnBSpline(int subsampling, double[][] controlPoints) {
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
	public static double[][] generateVelocitiesOnBSpline(int subsampling, double[][] controlPoints) {
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

	// calculate wire velocity, where + means letting out wire and - means reeling in wire
	public static double[] generateWireVelocity(double[][] vt, double[][] pt, double[] towerPosition) {
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

	/*// wrapper for myMotor.forward() so I don't get confused about what + or forward means
	// if you let out line at a negative speed, you will reel in line
	private void letOutLine() {
		myMotor.forward();
	}*/

	/*// wrapper for myMotor.backward() so I don't get confused about what - or backward means
	// if you reel in line at a negative speed, you will let out in line
	private void reelInLine() {
		myMotor.backward();
	}*/

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

	public OrbiterManagerTester() {
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
		// !! NEW !!
		mountPoint = new double[] {0D, 0D, 40D};		// uhhhhh idk. This should be encoded somewhere somehow. Also, would studs be a more convenient unit?
		allMountPoints = new double[][] {
			{ 0D,  0D, 40D},
			{40D,  0D, 40D},
			{ 0D, 40D, 40D},
			{40D, 40D, 40D}
		};
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
		int subsampling = 100;
		double[][] splinePoints = generatePointsOnBSpline(subsampling, controlPoints);
		double[][] splineVelocities = generateVelocitiesOnBSpline(subsampling, controlPoints);
		// TODO: so I think there's two ways to do this:
		// 1. calculate the spline for all towers, then normalize so that the fastest velocity between any two points is some cap
		// 2. calculate the spline for all towers, then iteratively subsample until the max velocity is below some cap
		// I think I prefer the speed normalization technique? Either way, we need to know the worst-case (fastest) speed for all the spools.
		double[][] allWireVelocities = new double[allMountPoints.length][];
		for (int i = 0; i < allMountPoints.length; i++) {
			allWireVelocities[i] = generateWireVelocity(splineVelocities, splinePoints, allMountPoints[i]);
		}
		// TODO: traverse all wire velocities and normalize them so that their units make sense
		// it also probaly makes sense to impose a minimum velocity since if the motor power gets too close to 0 it tends to not work
	}

	// will do one cycle of starting and stopping, then exit
	/*public void run() {
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
	}*/

	public static void main(String[] args)
	{
		// So this is just the test script so I can put whatever I want here.
		// I think it'll be a good idea for me to type out some utility functions.
		// Like... if I print out a spline, do the points match up with a similar spline from my python?
		double[][] controlPoints = new double[][] {
			{20D, 20D, 0D},
			{20D, 20D, 0D},
			{20D, 20D, 0D},
			{20D, 20D, 10D},
			{20D, 20D, 10D},
			{20D, 20D, 10D}
		};
		int subsampling = 10;
		double[][] allMountPoints = new double[][] {
			{ 0D,  0D, 40D},
			{40D,  0D, 40D},
			{ 0D, 40D, 40D},
			{40D, 40D, 40D}
		};
		double[][] splinePoints = generatePointsOnBSpline(subsampling, controlPoints);
		double[][] splineVelocities = generateVelocitiesOnBSpline(subsampling, controlPoints);
		double[][] allWireVelocities = new double[allMountPoints.length][];
		for (int i = 0; i < allMountPoints.length; i++) {
			allWireVelocities[i] = generateWireVelocity(splineVelocities, splinePoints, allMountPoints[i]);
		}
		double maxAllowedMotorSpeed = 99.0;
		double nominalMaxWireVelocity = 0.0;
		for (int i = 0; i < allWireVelocities.length; i++) {
			for (int j = 0; j < allWireVelocities[0].length; j++) {
				if (Math.abs(allWireVelocities[i][j]) > nominalMaxWireVelocity) {
					nominalMaxWireVelocity = Math.abs(allWireVelocities[i][j]);
				}
			}
		}
		int[][] adjustedWireVelocities = new int[allWireVelocities.length][allWireVelocities[0].length];
		double adjustmentMultiplier = maxAllowedMotorSpeed / nominalMaxWireVelocity;
		for (int i = 0; i < allWireVelocities.length; i++) {
			for (int j = 0; j < allWireVelocities[i].length; j++) {
				adjustedWireVelocities[i][j] = (int)(adjustmentMultiplier * allWireVelocities[i][j]);
			}
		}
		System.out.println(String.format("%s", Arrays.deepToString(splinePoints).replace(", [", ",\n [")));
		System.out.println(String.format("%s", Arrays.deepToString(splineVelocities).replace(", [", ",\n [")));
		System.out.println(String.format("%s", Arrays.deepToString(adjustedWireVelocities).replace(", [", ",\n [")));
	}
}

