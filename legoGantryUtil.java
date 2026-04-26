import java.util.ArrayList;

// calculate 3D Euclidean distance
public double distance(double[] xyz1, double[] xyz2) {
    double dx = xyz2[0] - xyz1[0];
    double dy = xyz2[1] - xyz1[1];
    double dz = xyz2[2] - xyz1[2];
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

// calculate wire length between a tower and the end effector
public double calculateWireLength(double[] towerPosition, double[] endEffectorPosition) {
    return distance(endEffectorPosition, towerPosition);
}

// calculate wire length change for a single tower
public double calculateWireLengthChange(double[] towerPosition, double[] currentPosition, double[] destinationPosition) {
    double currentWireLength = calculateWireLength(towerPosition, currentPosition);
    double destinationWireLength = calculateWireLength(towerPosition, destinationPosition);
    return destinationWireLength - currentWireLength;
}

// linearly interpolate between a and b, where p is between 0 and 1 inclusive
public double lerp1d(double a, double b, double p) {
    return (1 - p) * a + p * b;
}

// lerp n-dimensional vectors v1 and v2, where p is between 0 and 1 inclusive
// v1.length == v2.length must be true
public double[] lerpnd(double[] v1, double v2, double p) {
    if (v1.length != v2.length) {
        return null;
    }
    double[] retval = new double[v1.length];
    for (int i = 0; i < retval.length; i++) {
        retval[i] = lerp1d(v1[i], v2[i], p);
    }
    return retval;
}

public double[][] matmul(double[] a, double[] b) {
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

public double[] vectorDot(double[] v1, double[] v2) {
    double vdot = 0.0;
    for (int i = 0; i < v1.length; i++) {
        vdot = vdot + v1[i] * v2[i];
    }
    return vdot;
}

public double[] vectorSum(double[] v1, double[] v2) { 
    double[] vsum = new double[v1.length];
    for (int i = 0; i < v1.length; i++) {
        vsum[i] = v1[i] + v2[i];
    }
    return vsum;
}

public double[] vectorDifference(double[] v1, double[] v2) {
    double[] vdiff = new double[v1.length];
    for (int i = 0; i < v1.length; i++) {
        vdiff[i] = v1[i] - v2[i];
    }
    return vdiff;
}

public double vectorMagnitude(double[] v) {
    double magnitudeSquared = 0;
    for (int i = 0; i < v.length; i++) {
        magnitudeSquared = magnitudeSquared + v[i] * v[i];
    }
    return Math.sqrt(magnitudeSquared);
}

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
public double[] generatePointsOnBSpline(double subsampling, double[][] controlPoints) {
    ArrayList<double[]> splineXYZ = new ArrayList<double[]>();
    double[] t = new double[controlPoints.length * subsampling + 1];
    for (int i = 0; i < t.length; i++) {
        t[i] = lerp1d(0.0, controlPoints.length, i / t.length);
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
            double[] polynomialTerms = {1, u, u * u, u * u * u};
            double[][] points = new double[4][1];
            for (int k = i - 1; k < i + 3; k++) {
                points = [controlPoints[k]];
            }
            double[][] curveMat = matmul(polynomialTerms, matmul(characteristic_matrix, points));
            double[] curvePoint = new double[curveMat.length];
            for (int k = 0; k < curvePoint.length; k++) {
                curvePoint[k] = curveMat[k][0];
            }
            splineXYZ.add(curvePoint);
        }
    }
    return splineXYZ;
}

// Generate velocities on a b-spline, clamping to end control points
public double[] generateVelocitiesOnBSpline(double subsampling, double[][] controlPoints) {
    ArrayList<double[]> splineXYZ = new ArrayList<double[]>();
    double[] t = new double[controlPoints.length * subsampling + 1];
    for (int i = 0; i < t.length; i++) {
        t[i] = lerp1d(0.0, controlPoints.length, i / t.length);
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
            double[] polynomialTerms = {0, 1, 2 * u, 3 * u * u};
            double[][] points = new double[4][1];
            for (int k = i - 1; k < i + 3; k++) {
                points = [controlPoints[k]];
            }
            double[][] curveMat = matmul(polynomialTerms, matmul(characteristic_matrix, points));
            double[] curvePoint = new double[curveMat.length];
            for (int k = 0; k < curvePoint.length; k++) {
                curvePoint[k] = curveMat[k][0];
            }
            splineXYZ.add(curvePoint);
        }
    }
    return splineXYZ;
}

// calculate wire velocity
public double[] generateWireVelocity(double[][] vt, double[][] pt, double[] towerPosition) {
    double[] wireVelocities = new double[vt.length];
    for (int i = 0; i < vt.length; i++) {
        double[] endEffectorPosition = pt[i];
        double[] endEffectorVelocity = vt[i];
        double[] wireVector = vectorDifference(endEffectorPosition, towerPosition);
        double[] wireDirection = normalizeVector();
        double[] velocityAlongWire3d = vectorDot(vectorDot(endEffectorVelocity, wireDirection), wireDirection);
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