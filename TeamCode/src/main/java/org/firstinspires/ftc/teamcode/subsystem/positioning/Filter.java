package org.firstinspires.ftc.teamcode.subsystem.positioning;


import org.apache.commons.math3.linear.RealVector;

import java.util.List;

public class Filter {
	private boolean initialized;
	private boolean useControl;
	private boolean useDynamicProcessNoiseCovariance;
	private double lastMeasurementTime;
	private double latestControlTime;
	private double controlTimeout;
	private double sensorTimeout;
	// x, y, heading...
	private RealVector controlUpdateVector;
	private RealVector accelerationGains;
	private RealVector accelerationLimits;
	private RealVector decelerationGains;
	private RealVector decelerationLimits;
	private RealVector controlAcceleration;
	private RealVector latestControl;
	private List predictedState;
	private List state;
	private List<List> covarianceEpsilon;
	private List<List> dynamicProcessNoiseCovariance;
	private List<List> estimateErrorCovariance;
	private List<List> identity;
	private List<List> processnoiseCovariance;
	private List<List> transferFunction;
	private List<List> transferFunctionJacobian;

	public Filter() {

	}

	
}
