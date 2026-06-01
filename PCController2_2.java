package com.stan;


import java.util.Map;
import java.util.Set;
import java.util.HashMap;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.bluetooth.*;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.EOFException;
import java.io.IOException;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JTextField;
import javax.swing.border.Border;

import javax.swing.GroupLayout;
import javax.swing.*;
import java.awt.Color;
import java.util.regex.Pattern;

import lejos.pc.comm.*;
import lejos.util.Delay;


// this class runs everything
// it will create a separate thread for each mount point to manage communication 
// it will establish one thread to manage overall state of the system
public class PCController2_2 {

	private static int 			debug   	= 1;
	private static boolean		connected 	= false;
	private static boolean		finished 	= false;
	
	// basic configuration
	private static int			slaveCount 	= 4;				// how many nxts we actually want to use
	private static String[][] 	nxts = {
										  {"NXT1","00:16:53:09:E7:6E"}
										, {"NXT2","00:16:53:11:96:74"}
										, {"NXT3","00:16:53:0E:81:F6"}
										, {"NXT4","00:16:53:0A:BE:93"}
									};
	
	// basic configuration parameters
	private static 	enum CTRLST  {CTRLINIT, CTRLCONF, CTRLCONNG, PARKED, RAISING, STOPPED, STARTING, ORBITING, STOPPING, LOWERING, HALT};  // valid states for PC controller
	private static 	CTRLST 				ctrlState;						// current state of the orbiter
	private static 	float				hag, mps, ods, orb;				// config params in float form
	private static  int					sst, tto;						// configuration in integer form

	private        	HashMap<String, MountPoint>	mtPts;
	private static	enum MPST					{MPPREINIT, MPINIT, MPCONNG, MPCONN, MPCONFG, MPCONF, MPCALG, MPCAL, MPATTG, 
												MPPARKED, MPRAISING, MPSTOPPED, MPSTARTING, MPORBITING, MPSTOPPING, MPLOWER, MPHALT}; 
	private 		MPST						mpState;						// current joint state of mount points
	private 	   	HashMap<MPST, Integer> 		mtPtStates;
	
	// UI configurations - create at controller level just to make it easy
	private Border 				redBorder;
	private Border 				defaultBorder;
	private static JLabel 		ctrlStatus, mpStatus;
	private static JButton		initLock, initUnlock, orbitStart, orbitStop, orbitRaise, orbitLower, orbitHalt;
	private static JLabel		hagLabel, mpsLabel, odsLabel, orbLabel, sstLabel, ttoLabel;
	private static JLabel 		pcCtrlStatusLabel, mtPtStatusLbl;
	private static JTextField	heightAboveGround, mountPointSpacing, orbiterDipSpacing, orbitRadius, startStopTime, timeToOrbit;
	private        HashMap<String, MtPtDisplay> 	mtPtDisplays;
	
	// constructor
	// initializes variables and calls screenInit to build the UI
	public PCController2_2() {
		// do whatever setup is required

		// initialize variables 
		mtPts 		 = new HashMap<String,MountPoint>();		// holds the individual objects managing the mount points
		mtPtDisplays = new HashMap<String,MtPtDisplay>();		// holds the individual objects managing the mount point displays
		mtPtStates   = new HashMap<MPST, Integer>();

		// initialize mount points
		MountPoint tmpMtPt;
		for (int i=0; i < nxts.length; i++) {
			// create the mount point
			if(debug>0) System.out.println("initializing "+nxts[i][0]+".");
			tmpMtPt = new MountPoint(nxts[i][0], nxts[i][1], i < slaveCount? true:false);
			
			// add it to the list of mount points
			mtPts.put(nxts[i][0], tmpMtPt);
			if(debug>0) System.out.println(mtPts.get(nxts[i][0]).getName());
		}
		
		// set up variables for handling mount point states
		// initialize the overall states
		mpState	= MPST.MPPREINIT;

		// initialize the mount point state structure
		for (MPST mpst : MPST.values()) { 
		    mtPtStates.put(mpst, new Integer(0));
        } 
		if(debug>0) printMountPointStates();
		
		// initialize Controller UI components
		hagLabel	 = new JLabel("HAG:");
		mpsLabel	 = new JLabel("MPS:"); 
		odsLabel	 = new JLabel("ODS:"); 
		orbLabel	 = new JLabel("ORB:");
		sstLabel	 = new JLabel("SST:"); 
		ttoLabel	 = new JLabel("TTO:"); 
		
		heightAboveGround = new JTextField();
		mountPointSpacing = new JTextField(); 
		orbiterDipSpacing = new JTextField();
		orbitRadius		  = new JTextField();   
		startStopTime	  = new JTextField();
		timeToOrbit		  = new JTextField();
		pcCtrlStatusLabel = new JLabel("Controller: ");
		mtPtStatusLbl	  = new JLabel("Mount Points: ");
		ctrlStatus 	 	  = new JLabel("Not Initialized");
		mpStatus 	 	  = new JLabel("Initializing");
		initLock	 	  = new JButton("Lock  ");
		initUnlock	 	  = new JButton("Unlock");
		orbitStart   	  = new JButton("Start");
		orbitStop    	  = new JButton("Stop");
		orbitRaise   	  = new JButton("Raise");
		orbitLower   	  = new JButton("Lower");
		orbitHalt    	  = new JButton("Halt");
		
		// initialize Mount Point UI components
		MtPtDisplay tmpMtPtDisplay;
		for (int i=0; i < nxts.length; i++) {
			// create the mount point
			if(debug>0) System.out.println("initializing UI for "+nxts[i][0]+".");
			tmpMtPtDisplay = new MtPtDisplay(nxts[i][0], i < slaveCount? true:false);
			
			// add it to the list of mount points
			mtPtDisplays.put(nxts[i][0], tmpMtPtDisplay);
			if(debug>0) System.out.println(mtPts.get(nxts[i][0]).getName());
		}
	}
	
	// class for managing state / comms with nxt
	private class MountPoint extends Thread {
		// internal variables
		private String				mpName;
		private String				mpAddr;

		// state
		private MPST				mpState;
		private boolean				isConnected;		
		private boolean				isActive;
		private boolean				stateChanged;

		// communication variables
		private NXTComm				nxtComm;
		private NXTInfo 			nxtInfo;
		private DataOutputStream	nxtOutput;
		private DataInputStream		nxtInput;

		// constructor
		private MountPoint(String name, String addr, boolean active) {			
			mpName		 = name;
			mpAddr		 = addr;
			isActive	 = active;	
			isConnected  = false;
			stateChanged = false;
			mpState		 = MPST.MPPREINIT;
			
			// since this is a thread, set the name of the thread to equal
			// the mount point name, so we can use getName()
			setName(name);
			
		}

		private MPST getMtPtState() {
			return mpState;
		}
		
		private boolean hasChangedState() {
			if(stateChanged) {
				stateChanged = false;
				return true;
			}
			else {
				return false;
			}
		}

		public void write(float val) {
			if(debug>0) System.out.println("\nwriting int value "+val);
			try {
				nxtOutput.writeFloat(val);
				nxtOutput.flush();
			} 
			catch(Exception e) {
				e.printStackTrace();
			}
		}
		
		public void write(int val) {
			if(debug>0) System.out.println("\nwriting int value "+val);
			try {
				nxtOutput.writeInt(val);
				nxtOutput.flush();
			} 
			catch(Exception e) {
				e.printStackTrace();
			}
		}
		
		public void write(CTRLST ctrlState) {
			write(ctrlState.toString());
		}

		public void write(String val) {
			if(debug>0) System.out.println("\nwriting string value "+val);
			try {
				nxtOutput.writeUTF(val);
				nxtOutput.flush();
			} 
			catch(Exception e) {
				e.printStackTrace();
			}
		}
		
		// start up the listener for the nxt
//		private static	enum MPST					{MPPREINIT, MPINIT, MPCONNG, MPCONN, MPCONFG, MPCONF, MPCALG, MPCAL, MPATTG, 
//													MPPARKED, MPRAISING, MPSTOPPED, MPSTARTING, MPORBITING, MPSTOPPING, MPLOWERING, MPHALT}; 
		public void run() {
			if(isActive) {
				/*
				 *  part 1 - set up mount point, from pre-init through parked
				 */
				if(debug>0) System.out.println("\nmount point "+mpName+" is running");
				// flag for saying when it is time to stop
				boolean timeToStop = false;
				
				// flag for knowing when we care (change of state)
				MPST	newState   = mpState;
				if(debug>0) System.out.println("\nmount point "+mpName+" state: "+newState);
				
				// set up the connection
				connectMountPoint();
				
				// read from connection in a loop
				// change stuff as needed
				while (! timeToStop) {
					try {
						if(debug>0) System.out.println("\nnxtConn "+mpName+": reading");
						
						// read the input
						newState = MPST.valueOf(nxtInput.readUTF());
						if(debug>0) System.out.println("\nnxtConn "+mpName+": read "+newState);
						
						// do stuff with it
						if(newState != mpState) {
							if(debug>0) System.out.println("\nnxtConn "+mpName+": switch ("+newState+")");
							mpState 	 = newState;		
							stateChanged = true;
							if(mpState == MPST.MPHALT) {
								timeToStop = true;
							}
						}
					}
					catch(EOFException eof) {
						if(debug>0) System.out.println("\n"+mpName+": Lost connection!");
						if(debug>0) System.out.println("\n"+mpName+": Calling halt");
						doHalt();
						timeToStop = true;
					}
					catch(IOException eof) {
						if(debug>0) System.out.println("\n"+mpName+": Lost connection!");
						if(debug>0) System.out.println("\n"+mpName+": Calling halt");
						doHalt();
						timeToStop = true;
					}
					catch(Exception e) {
						e.printStackTrace();
					}
				}	
				// thread is done - exit
				if(debug>0) System.out.println("\nmount point "+mpName+" is stopping");
			}
			else {
				if(debug>0) System.out.println("\nmount point "+mpName+" is inactive");
			}
		}
		
		// initialize the NXT communications stuff
		private void connectMountPoint() {
			mpState 	 = MPST.MPINIT;
			stateChanged = true;
			if(debug>0) System.out.println("\nmount point "+mpName+" state: "+mpState);
			
			try {					
				// create the connections first
				if (debug > 0) System.out.println("\ngetting nxtComm for " + mpName);
				nxtComm = NXTCommFactory.createNXTComm(NXTCommFactory.BLUETOOTH);
				
				if (debug > 0) System.out.println("\ngetting nxtInfo for " + mpName);
				nxtInfo = new NXTInfo(NXTCommFactory.BLUETOOTH, mpName, mpAddr); 
	
				while(! isConnected) {
					try {
						if (debug > 0) System.out.println("\nopening nxtComm for " + mpName);
						while(! nxtComm.open(nxtInfo)) continue;
						isConnected = true;
					}
					catch(NXTCommException nce) {
						if(debug>0) System.out.println("\ngot error in connect "+nce.toString());
					}
					catch(Exception e) {
						e.printStackTrace();
					}
				}
				
				// awesome!
				// once we are connected, create the data streams
				if (debug > 0) System.out.println("\ngetting nxtInput for " + mpName);
				nxtInput = new DataInputStream(nxtComm.getInputStream());
	
				if (debug > 0) System.out.println("\ngetting nxtOutput for " + mpName);
				nxtOutput = new DataOutputStream(nxtComm.getOutputStream());

				// update the state
				mpState = MPST.MPCONN;
				stateChanged = true;
				if(debug>0) System.out.println("\nmount point "+mpName+" state: "+mpState);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}
	
	// print out current master view of mount point state
	private void printMountPointStates() {
		for (MPST mpst : MPST.values()) { 
		    System.out.print(mpst.name() +": "+mtPtStates.get(mpst).toString()+", ");
        } 	
		System.out.println();
	}
	
	// the view portion for mount point status
	// the view portion for NXT status
	// we will create these to handle the displays
	// for the max # of mount points
	private class MtPtDisplay {
		// variables
		private JLabel 	mtPtNameLbl;
		private JLabel 	mtPtConnLbl;
		private JLabel	mtPtStateLbl;
		
		// constructor
		private MtPtDisplay(String name, boolean isActive) {			
			mtPtNameLbl 	= new JLabel(name);
			if(isActive) {
				mtPtStateLbl 	= new JLabel("pending");
				mtPtConnLbl 	= new JLabel("pending");
			}
			else {
				mtPtStateLbl 	= new JLabel("inactive");
				mtPtConnLbl 	= new JLabel("inactive");
			}
		}

		private JLabel getMtPtNameLbl() {
			return mtPtNameLbl;
		}

		private JLabel getMtPtConnLbl() {
			return mtPtConnLbl;
		}

		private JLabel getMtPtStateLbl() {
			return mtPtStateLbl;
		}

		private void setMtPtStateLbl(String s) {
			mtPtStateLbl.setText(s);
		}
		
		private void setMtPtConnLbl(String s) {
			mtPtConnLbl.setText(s);
		}
	}
	

	// send the configuration info to the mount points
	// called when config is locked and mount points are connected
	public void configMtPts(float hag, float mps, float orb, float ods, int sst, int tto) {
		try {
			if(debug>0) System.out.println("writing config to mount points");
			for (Map.Entry<String, MountPoint> mp : mtPts.entrySet()) {
				if(mp.getValue().isActive) {
					if(debug>0) System.out.println("writing config to nxt "+mp.getKey());
					mp.getValue().write(hag);
					mp.getValue().write(mps);
					mp.getValue().write(orb);		
					mp.getValue().write(ods);		
					mp.getValue().write(sst);		
					mp.getValue().write(tto);		
					if(debug>0) System.out.println("config to nxt "+mp.getKey()+" written");
				}
				else {
					if(debug>0) System.out.println(mp.getKey()+" is inactive");
				}
			}
		}
		catch(Exception e) {
			e.printStackTrace();
		}
	}
	
	// tell the caller what the current state of the mount points is
	private MPST getMtPtStates() {
		// MPPREINIT, MPINIT, MPCONN, MPCONF, MPCAL, MPATTACH, MPPARK, MPRAISE, MPSTART, MPORBIT, MPSTOP, MPLOW, MPHALT
		// we will store this value locally (MountPoints), but update every time this function is called
		// until we get to MPPARK, we will clear all the states 
		
		if(debug>2) System.out.println("getMtPtStates: current state is "+mpState);
		if(debug>2) printMountPointStates();
		
		for (MPST mpst : MPST.values()) { 
		    mtPtStates.put(mpst, new Integer(0));
        } 
		if(debug > 2) printMountPointStates();
		

		// now update the values in mtPtStates
		for(Map.Entry<String, MountPoint> mp : mtPts.entrySet()) {
			if(debug > 2) printMountPointStates();
			if(debug>2) System.out.println("mount point "+mp.getValue().getName()+": updating values in mtPtStates for "+mp.getValue().getMtPtState());
			if(debug > 2) printMountPointStates();
			if(mp.getValue().isActive) {
				if(debug>2) System.out.println("this one is active");
				switch (mp.getValue().getMtPtState()) {
					case MPPREINIT:	mtPtStates.put(MPST.MPPREINIT, new Integer(mtPtStates.get(MPST.MPPREINIT).intValue()+1)); break; 
					case MPINIT:	mtPtStates.put(MPST.MPINIT, new Integer(mtPtStates.get(MPST.MPINIT).intValue()+1)); break;
					case MPCONNG: 	mtPtStates.put(MPST.MPCONNG, new Integer(mtPtStates.get(MPST.MPCONNG).intValue()+1)); break;
					case MPCONN: 	mtPtStates.put(MPST.MPCONN, new Integer(mtPtStates.get(MPST.MPCONN).intValue()+1)); break;
					case MPCONFG:	mtPtStates.put(MPST.MPCONFG, new Integer(mtPtStates.get(MPST.MPCONFG).intValue()+1)); break;	
					case MPCONF:	mtPtStates.put(MPST.MPCONF, new Integer(mtPtStates.get(MPST.MPCONF).intValue()+1)); break;	
					case MPCALG:	mtPtStates.put(MPST.MPCALG, new Integer(mtPtStates.get(MPST.MPCALG).intValue()+1)); break;
					case MPCAL:		mtPtStates.put(MPST.MPCAL, new Integer(mtPtStates.get(MPST.MPCAL).intValue()+1)); break;
					case MPATTG:	mtPtStates.put(MPST.MPATTG, new Integer(mtPtStates.get(MPST.MPATTG).intValue()+1)); break;	
					case MPPARKED:	mtPtStates.put(MPST.MPPARKED, new Integer(mtPtStates.get(MPST.MPPARKED).intValue()+1)); break;
					case MPRAISING:	mtPtStates.put(MPST.MPRAISING, new Integer(mtPtStates.get(MPST.MPRAISING).intValue()+1)); break;	
					case MPSTOPPED:	mtPtStates.put(MPST.MPSTOPPED, new Integer(mtPtStates.get(MPST.MPSTOPPED).intValue()+1)); break;		
					case MPSTARTING:	mtPtStates.put(MPST.MPSTARTING, new Integer(mtPtStates.get(MPST.MPSTARTING).intValue()+1)); break;	
					case MPORBITING:	mtPtStates.put(MPST.MPORBITING, new Integer(mtPtStates.get(MPST.MPORBITING).intValue()+1)); break;
					case MPSTOPPING:	mtPtStates.put(MPST.MPSTOPPING, new Integer(mtPtStates.get(MPST.MPSTOPPING).intValue()+1)); break;		
					case MPLOWER:	mtPtStates.put(MPST.MPLOWER, new Integer(mtPtStates.get(MPST.MPLOWER).intValue()+1)); break;	
					case MPHALT:	mtPtStates.put(MPST.MPHALT, new Integer(mtPtStates.get(MPST.MPHALT).intValue()+1)); break;	
					default:				
				}
			}
			if(debug>2) printMountPointStates();
		}
		
		// now that the mtPtStates structure is updated, calculate mpState
		MPST tempMpState = MPST.MPPREINIT;
		
		// loop through each state
		for (MPST mpst : MPST.values()) { 
			// if we are higher than current state and all the slaves are at the higher state, update
			if(debug>2) System.out.println("in loop: mpState = "+mpState+", value is "+mpst+", count is "+mtPtStates.get(mpst));
		    if(mtPtStates.get(mpst) == slaveCount) {
		    	tempMpState = mpst; 
		    	if(debug>2) System.out.println("tempMpState is now "+tempMpState);
		    }
        } 
		
		if(mpState != tempMpState) {
			if(debug>0) System.out.println("current mpState = "+mpState+", new mpState = "+tempMpState);
			mpState = tempMpState;
			if(debug>0) printMountPointStates();
		};
		
		return mpState;
	}

	// set up the UI
	private void initScreen() {
		// UI stuff
		JFrame f = new JFrame("PC Controller");
		
		GroupLayout layout = new GroupLayout(f.getContentPane());
		f.getContentPane().setLayout(layout);
		f.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);

		// set the action listeners for the buttons
		initLock.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				doInitLock();
			}
		});		
		initUnlock.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				doInitUnlock();
			}
		});		
		orbitStart.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				doStarting();
			}
		});
		orbitStop.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				doStopping();
			}
		});
		orbitRaise.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				beginRaising();
			}
		});
		orbitLower.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				beginLowering();
			}
		});
		orbitHalt.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				doHalt();
			}
		});
		
		layout.setAutoCreateGaps(true);
		layout.setAutoCreateContainerGaps(true);
		
		// Hard-code the layout
		layout.setHorizontalGroup(layout.createParallelGroup()
				// status row
				.addGroup(layout.createSequentialGroup()
						.addComponent(hagLabel, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
						.addComponent(heightAboveGround, GroupLayout.PREFERRED_SIZE, 50, Short.MAX_VALUE)
						.addComponent(mpsLabel, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
						.addComponent(mountPointSpacing, GroupLayout.PREFERRED_SIZE, 50, Short.MAX_VALUE)
						.addComponent(odsLabel, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
						.addComponent(orbiterDipSpacing, GroupLayout.PREFERRED_SIZE, 50, Short.MAX_VALUE)
						)
				.addGroup(layout.createSequentialGroup()
						.addComponent(orbLabel, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
						.addComponent(orbitRadius, GroupLayout.PREFERRED_SIZE, 50, Short.MAX_VALUE)
						.addComponent(sstLabel, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
						.addComponent(startStopTime, GroupLayout.PREFERRED_SIZE, 50, Short.MAX_VALUE)
						.addComponent(ttoLabel, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
						.addComponent(timeToOrbit, GroupLayout.PREFERRED_SIZE, 50, Short.MAX_VALUE)
						)
				.addGroup(layout.createSequentialGroup()
						.addComponent(initLock, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
						.addComponent(initUnlock, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
						)
				.addGroup(layout.createSequentialGroup()
						.addComponent(pcCtrlStatusLabel, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
						.addComponent(ctrlStatus, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
						.addComponent(mtPtStatusLbl, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
						.addComponent(mpStatus, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
						)
				// nxt status rows
				.addGroup(layout.createSequentialGroup()
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.CENTER)
								.addComponent(mtPtDisplays.get("NXT1").getMtPtNameLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT2").getMtPtNameLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT3").getMtPtNameLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT4").getMtPtNameLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.CENTER)
								.addComponent(mtPtDisplays.get("NXT1").getMtPtConnLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT2").getMtPtConnLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT3").getMtPtConnLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT4").getMtPtConnLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.CENTER)
								.addComponent(mtPtDisplays.get("NXT1").getMtPtStateLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT2").getMtPtStateLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT3").getMtPtStateLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT4").getMtPtStateLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								)
						)
				// regular button row
				.addGroup(layout.createSequentialGroup()
						.addComponent(orbitRaise, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
						.addComponent(orbitStart, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
						.addComponent(orbitStop, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
						.addComponent(orbitLower, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
						)
				// halt button row
				.addComponent(orbitHalt, 0, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
		);
		
		layout.setVerticalGroup(layout.createSequentialGroup()
				// variables
				.addGroup(layout.createParallelGroup()
						.addComponent(hagLabel)
						.addComponent(heightAboveGround)
						.addComponent(mpsLabel)
						.addComponent(mountPointSpacing)
						.addComponent(odsLabel)
						.addComponent(orbiterDipSpacing)
						)
				.addGroup(layout.createParallelGroup()
						.addComponent(orbLabel)
						.addComponent(orbitRadius)
						.addComponent(sstLabel)
						.addComponent(startStopTime)
						.addComponent(ttoLabel)
						.addComponent(timeToOrbit)
						)
				// variable controls
				.addGroup(layout.createParallelGroup()
						.addComponent(initLock)
						.addComponent(initUnlock)
						)				
				// status row
				.addGroup(layout.createParallelGroup(GroupLayout.Alignment.CENTER)
				.addComponent(ctrlStatus, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
				.addComponent(pcCtrlStatusLabel, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
				.addComponent(mpStatus, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
				.addComponent(mtPtStatusLbl, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
					)
				// nxt status rows
				.addGroup(layout.createParallelGroup(GroupLayout.Alignment.CENTER)
						.addGroup(layout.createSequentialGroup()
								.addComponent(mtPtDisplays.get("NXT1").getMtPtNameLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT2").getMtPtNameLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT3").getMtPtNameLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT4").getMtPtNameLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								)
						.addGroup(layout.createSequentialGroup()
								.addComponent(mtPtDisplays.get("NXT1").getMtPtConnLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT2").getMtPtConnLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT3").getMtPtConnLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT4").getMtPtConnLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								)
						.addGroup(layout.createSequentialGroup()
								.addComponent(mtPtDisplays.get("NXT1").getMtPtStateLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT2").getMtPtStateLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT3").getMtPtStateLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								.addComponent(mtPtDisplays.get("NXT4").getMtPtStateLbl(), GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
								)
					)
				 // regular button row
				.addGroup(layout.createParallelGroup()
					.addComponent(orbitRaise, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
					.addComponent(orbitStart, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
					.addComponent(orbitStop, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
					.addComponent(orbitLower, GroupLayout.DEFAULT_SIZE, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
				)
				// halt button row
				.addComponent(orbitHalt, 0, GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
		);

		redBorder = BorderFactory.createMatteBorder(3, 3, 3, 3, Color.red);
		defaultBorder = heightAboveGround.getBorder();
		
		// display the screen
		f.pack();
		f.setVisible(true);
	}
	
	// set everything to initial state
	private void initCtrl() {
		// initialize state first so we know where we are
		ctrlStatus.setText("Initialized");
		ctrlState   = CTRLST.CTRLINIT;
		
		// set buttons to match state
		initLock.setEnabled(true);
		initUnlock.setEnabled(false);
		orbitStart.setEnabled(false);
		orbitStop.setEnabled(false);
		orbitRaise.setEnabled(false);
		orbitLower.setEnabled(false);
		orbitHalt.setEnabled(false);			
				
		// build the screen
		initScreen();
	}
	
	// determine if the measurement input is valid (numeric, non-blank)
    private boolean isValidInput(String str) {
    	if(debug>0) System.out.println("checking '"+str+"': result is "+Pattern.matches("[0-9]+.?[0-9]*",str));
    	
        if(str != null && !str.isEmpty()) {
        	if(Pattern.matches("[0-9]+\\.?[0-9]*",str))		// numeric
        		return true;
        }
        return false;
    }
    
    // execute all the changes needed when Lock is pressed
	private void doInitLock() {	
		// set state and conditions when initLock is pressed
		// if everything is valid, lock it
		// if not, don't allow the lock, but provide notification
		// check that we have values first
		if(   isValidInput(heightAboveGround.getText()) && isValidInput(mountPointSpacing.getText()) && isValidInput(orbiterDipSpacing.getText()) 
		   && isValidInput(orbitRadius.getText()) && isValidInput(startStopTime.getText()) && isValidInput(timeToOrbit.getText())
		   && Float.parseFloat(orbitRadius.getText()) <= 0.85F * Float.parseFloat(mountPointSpacing.getText())) {
			// all good 			
			// lock the suckers
			heightAboveGround.setEditable(false);
			mountPointSpacing.setEditable(false);
			orbiterDipSpacing.setEditable(false);
			orbitRadius.setEditable(false);
			startStopTime.setEditable(false);
			timeToOrbit.setEditable(false);
			
			// set the values
			hag = Float.parseFloat(heightAboveGround.getText());
			mps = Float.parseFloat(mountPointSpacing.getText());
			ods = Float.parseFloat(orbiterDipSpacing.getText());
			orb = Float.parseFloat(orbitRadius.getText());
			sst = Integer.parseInt(startStopTime.getText());
			tto = Integer.parseInt(timeToOrbit.getText());
			
			// reset the borders if necessary
			if(heightAboveGround.getBorder() == redBorder) {
				heightAboveGround.setBorder(defaultBorder);
			}
			if (mountPointSpacing.getBorder() == redBorder) {
				mountPointSpacing.setBorder(defaultBorder);
			}
			if (orbiterDipSpacing.getBorder() == redBorder) {
				orbiterDipSpacing.setBorder(defaultBorder);
			}
			if (orbitRadius.getBorder() == redBorder) {
				orbitRadius.setBorder(defaultBorder);
			}
			if (startStopTime.getBorder() == redBorder) {
				startStopTime.setBorder(defaultBorder);
			}
			if (timeToOrbit.getBorder() == redBorder) {
				timeToOrbit.setBorder(defaultBorder);
			}
			if(debug>0) System.out.println("heightAboveGround = "+heightAboveGround.getText()+"; mountPointSpacing = "+mountPointSpacing.getText()+"; orbiterDipSpacing = "
											+orbiterDipSpacing.getText()+"; orbitRadius = "+orbitRadius.getText()+"; startStopTime = "+startStopTime.getText()+
											"; timeToOrbit = "+timeToOrbit.getText());
			if(debug>0) System.out.println("heightAboveGround = "+hag+"; mountPointSpacing = "+mps+"; orbiterDipSpacing = "+ods+"; orbitRadius = "+orb+"; sst = "+sst+"; tto = "+tto);
//			if(debug>0) System.out.println("hag length = "+heightAboveGround.getText().length()+"; mps length = "+mountPointSpacing.getText().length()+"; dip length = "
//											+orbiterDipSpacing.getText().length()+"; orbRadius length = "+orbitRadius.getText().length());			

			// change visibility on UI
			initLock.setEnabled(false);
			// if we are connected or configuring, we don't want to allow re-configuration
			// disable unlock
			if(mpState == MPST.MPCONN || mpState == MPST.MPCONFG) {
				initUnlock.setEnabled(false);
				orbitStart.setEnabled(false);
				orbitStop.setEnabled(false);
				orbitRaise.setEnabled(false);
				orbitLower.setEnabled(false);
				orbitHalt.setEnabled(true);
			}
			else {
				initUnlock.setEnabled(true);
				orbitStart.setEnabled(false);
				orbitStop.setEnabled(false);
				orbitRaise.setEnabled(false);
				orbitLower.setEnabled(false);
				orbitHalt.setEnabled(true);
			}
			
			// change state
			ctrlStatus.setText("Configured");
			ctrlState = CTRLST.CTRLCONF;
		
		}
		else {
			//something is wrong - provide notification
			// do not lock
			if(! isValidInput(heightAboveGround.getText())) {
				heightAboveGround.setBorder(redBorder);
				heightAboveGround.setEditable(true);
			}
			else {
				heightAboveGround.setBorder(defaultBorder);
			}
			if (! isValidInput(mountPointSpacing.getText())) {
				mountPointSpacing.setBorder(redBorder);
				mountPointSpacing.setEditable(true);
			}
			else {
				mountPointSpacing.setBorder(defaultBorder);
			}
			if (! isValidInput(orbiterDipSpacing.getText())) {
				orbiterDipSpacing.setBorder(redBorder);
				orbiterDipSpacing.setEditable(true);
			}
			else {
				orbiterDipSpacing.setBorder(defaultBorder);
			}
			if (! isValidInput(orbitRadius.getText())) {
				orbitRadius.setBorder(redBorder);
				orbitRadius.setEditable(true);
			}
			else if (isValidInput(orbitRadius.getText()) && isValidInput(mountPointSpacing.getText()) && 
					Integer.parseInt(orbitRadius.getText()) > 0.85 * Integer.parseInt(mountPointSpacing.getText())) {	// it may be valid, but we also want to make sure the orbiting size is smaller than the spacing
				orbitRadius.setBorder(redBorder);
				orbitRadius.setEditable(true);
			}
			else {
				orbitRadius.setBorder(defaultBorder);
			}
			if (! isValidInput(startStopTime.getText())) {
				startStopTime.setBorder(redBorder);
				startStopTime.setEditable(true);
			}
			else {
				startStopTime.setBorder(defaultBorder);
			}
			if (! isValidInput(timeToOrbit.getText())) {
				timeToOrbit.setBorder(redBorder);
				timeToOrbit.setEditable(true);
			}
			else {
				timeToOrbit.setBorder(defaultBorder);
			}
		}
	}
	
	// execute all the changes needed when Unlock is pressed
	private void doInitUnlock() {
		
		heightAboveGround.setEditable(true);
		mountPointSpacing.setEditable(true);
		orbiterDipSpacing.setEditable(true);
		orbitRadius.setEditable(true);
		startStopTime.setEditable(true);
		timeToOrbit.setEditable(true);
		
		// set state and conditions when initUnlock is pressed
		// this moves us back to controller init
		initLock.setEnabled(true);
		initUnlock.setEnabled(false);
		orbitStart.setEnabled(false);
		orbitStop.setEnabled(false);
		orbitRaise.setEnabled(false);
		orbitLower.setEnabled(false);
		orbitHalt.setEnabled(true);	
	}

	// execute the changes when Start is signaled - 
	// this is the starting state
	private void doStarting() {
		// should be stopped before we do this
		if(ctrlState == CTRLST.STOPPED) {
			// change controller state
			ctrlState = CTRLST.STARTING;

			// update the UI
			ctrlStatus.setText("Starting");
			orbitStart.setEnabled(false);
			orbitStop.setEnabled(true);		// it is possible to send stop immediately after start
			orbitRaise.setEnabled(false);
			orbitLower.setEnabled(false);
			orbitHalt.setEnabled(true);

			// notify mount points
			try {
				if(debug>0) System.out.println("sending 'start' to mount points");
				for (Map.Entry<String, MountPoint> mp : mtPts.entrySet()) {
					if(mp.getValue().isActive) {
						if(debug>0) System.out.println("writing 'start' to nxt "+mp.getKey());
						mp.getValue().write(CTRLST.STARTING);	
						if(debug>0) System.out.println("nxt "+mp.getKey()+": 'start' written");
					}
					else {
						if(debug>0) System.out.println(mp.getKey()+" is inactive");
					}
				}
			}
			catch(Exception e) {
				e.printStackTrace();
			}
		}
		else {
			problem("STARTING");
		}
	}
	
	// execute the changes when Stop is signaled - 
	// this is the stopping state
	private void doStopping() {
		if(ctrlState == CTRLST.STARTING || ctrlState == CTRLST.ORBITING) {
			// change controller state
			ctrlState = CTRLST.STOPPING;
			
			// update the UI
			ctrlStatus.setText("Stopping");
			orbitStart.setEnabled(false);
			orbitStop.setEnabled(false);
			orbitRaise.setEnabled(false);
			orbitLower.setEnabled(false);
			orbitHalt.setEnabled(true);
	
			// notify mount points
			try {
				if(debug>0) System.out.println("sending 'stop' to mount points");
				for (Map.Entry<String, MountPoint> mp : mtPts.entrySet()) {
					if(mp.getValue().isActive) {
						if(debug>0) System.out.println("writing 'stop' to nxt "+mp.getKey());
						mp.getValue().write(CTRLST.STOPPING);	
						if(debug>0) System.out.println("nxt "+mp.getKey()+": 'stop' written");
					}
					else {
						if(debug>0) System.out.println(mp.getKey()+" is inactive");
					}
				}
			}
			catch(Exception e) {
				e.printStackTrace();
			}
		}
		else {
			problem("doStopping: ctrlState = "+ctrlState);
		}
	}
	
	// execute the changes when Raise is signaled - 
	// this is the raising state
	private void beginRaising() {
		if(ctrlState == CTRLST.PARKED) {
			// change controller status
			ctrlState = CTRLST.RAISING;
			
			// update the UI
			ctrlStatus.setText("Raising");
			orbitStart.setEnabled(false);
			orbitStop.setEnabled(false);
			orbitRaise.setEnabled(false);
			orbitLower.setEnabled(false);
			orbitHalt.setEnabled(true);
			
			// notify mount points
			try {
				if(debug>0) System.out.println("sending 'raising' to mount points");
				for (Map.Entry<String, MountPoint> mp : mtPts.entrySet()) {
					if(mp.getValue().isActive) {
						if(debug>0) System.out.println("writing 'raising' to nxt "+mp.getKey());
						mp.getValue().write(CTRLST.RAISING);	
						if(debug>0) System.out.println("nxt "+mp.getKey()+": 'raising' written");
					}
					else {
						if(debug>0) System.out.println(mp.getKey()+" is inactive");
					}
				}
			}
			catch(Exception e) {
				e.printStackTrace();
			}
		}
		else {
			problem("beginRaising: ctrlState = "+ctrlState);
		}
	}
	
	// execute the changes when Lower is signaled - 
	// this is the lowering state
	private void beginLowering() {
		if(ctrlState == CTRLST.STOPPED) {
			// update controller state
			ctrlState = CTRLST.LOWERING;
	
			// update UI
			ctrlStatus.setText("Lowering");
			orbitStart.setEnabled(false);
			orbitStop.setEnabled(false);
			orbitRaise.setEnabled(false);
			orbitLower.setEnabled(false);
			orbitHalt.setEnabled(true);
			
			// notify mount points
			try {
				if(debug>0) System.out.println("sending 'lower' to mount points");
				for (Map.Entry<String, MountPoint> mp : mtPts.entrySet()) {
					if(mp.getValue().isActive) {
						if(debug>0) System.out.println("writing 'lower' to nxt "+mp.getKey());
						mp.getValue().write(CTRLST.LOWERING);	
						if(debug>0) System.out.println("nxt "+mp.getKey()+": 'lowering' written");
					}
					else {
						if(debug>0) System.out.println(mp.getKey()+" is inactive");
					}
				}
			}
			catch(Exception e) {
				e.printStackTrace();
			}
		}
		else {
			problem("doLowering: ctrlState = "+ctrlState);
		}
	}
	
	// execute all the changes needed when Halt is signaled
	// everything stops immediately, but we enter a halting state until we hear from mount points
	private void doHalt() {
		ctrlStatus.setText("Halting");
		ctrlState = CTRLST.HALT;
		
		// once we halt, all actions should cease
		initLock.setEnabled(false);
		initUnlock.setEnabled(false);
		orbitStart.setEnabled(false);
		orbitStop.setEnabled(false);
		orbitRaise.setEnabled(false);
		orbitLower.setEnabled(false);
		orbitHalt.setEnabled(false);

		// notify mount points
		try {
			if(debug>0) System.out.println("sending 'halt' to mount points");
			for (Map.Entry<String, MountPoint> mp : mtPts.entrySet()) {
				if(mp.getValue().isActive) {
					if(debug>0) System.out.println("writing 'halt' to nxt "+mp.getKey());
					mp.getValue().write(CTRLST.HALT);	
					if(debug>0) System.out.println("nxt "+mp.getKey()+": 'halt' written");
				}
				else {
					if(debug>0) System.out.println(mp.getKey()+" is inactive");
				}
			}
		}
		catch(Exception e) {
			e.printStackTrace();
		}
	}
	
	// do the stuff needed to address changes in state
	private void refreshStates() {
		boolean mpStateChanged = false;			// keep track of whether something has changed this time
		// update individual states first
		// this part is easy - just change the UI and give it friendly names
		for(Map.Entry<String, MountPoint> mp : mtPts.entrySet()) {
			if(mp.getValue().hasChangedState()) {
				mpStateChanged = true;
				switch(mp.getValue().getMtPtState()) {
				case	MPPREINIT:	mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Pre-initialized");	break;
				case	MPINIT:		mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Initialized"); 		break;
				case	MPCONNG:	mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Connecting "); 		
				case	MPCONN:		mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Connected  "); 		
									mtPtDisplays.get(mp.getKey()).setMtPtConnLbl("Connected   "); 		break;
				case	MPCONFG:	mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Configuring"); 		break;
				case	MPCONF:		mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Configured "); 		break;
				case	MPCALG:		mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Calibrating"); 		break;
				case	MPCAL:		mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Calibrated "); 		break;
				case	MPATTG:		mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Attaching  "); 		break;
				case	MPPARKED:	mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Parked     "); 		break;
				case	MPRAISING:	mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Raising    "); 		break;
				case	MPSTOPPED:	mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Stopped    "); 		break;
				case	MPSTARTING:	mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Starting   "); 		break;
				case	MPORBITING:	mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Orbiting   "); 		break;
				case	MPSTOPPING:	mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Stopping   "); 		break;
				case	MPLOWER:	mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Lowering   "); 		break;
				case	MPHALT:		mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("Halted     "); 		break;
					default:	mtPtDisplays.get(mp.getKey()).setMtPtStateLbl("beats me");
				}
			}
		}
		
		// get our global state for mount points and update UI / controller state as appropriate
		if(mpStateChanged) {
			mpState = getMtPtStates(); 
			switch(mpState) {
				case	MPPREINIT:	mpStatus.setText("Pre-initialized");
									if(debug>0) System.out.println("mppreinitialized");	
									break;
				case	MPINIT:		mpStatus.setText("Initialized"); 
									if(debug>0) System.out.println("mpinitialized");			
									break;
				case	MPCONNG:	mpStatus.setText("Connecting "); 
									if(debug>0) System.out.println("mpconnecting");			
									break;
				case	MPCONN:		mpStatus.setText("Connected  "); 
									if(debug>0) System.out.println("mpconnected");			
									break;
				case	MPCONFG:	mpStatus.setText("Configuring"); 
									if(debug>0) System.out.println("mpconfiguring");			
									break;
				case	MPCONF:		mpStatus.setText("Configured "); 	
									if(debug>0) System.out.println("mpconfigured");		
									break;
				case	MPCALG:		mpStatus.setText("Calibrating"); 	
									if(debug>0) System.out.println("mpcalibrating");		
									break;
				case	MPCAL:		mpStatus.setText("Calibrated "); 	
									if(debug>0) System.out.println("mpcalibrated");		
									break;
				case	MPATTG:		mpStatus.setText("Attaching  "); 	
									if(debug>0) System.out.println("mpattaching");		
									break;
				case	MPPARKED:	mpStatus.setText("Parked     "); 		
									if(ctrlState == CTRLST.LOWERING || ctrlState == CTRLST.CTRLCONF) {
										// update state
										ctrlState = CTRLST.PARKED;
										// update UI
										ctrlStatus.setText("Parked");
										orbitStart.setEnabled(false);
										orbitStop.setEnabled(false);
										orbitRaise.setEnabled(true);
										orbitLower.setEnabled(false);
										if(debug>0) System.out.println("mpparked");
									}	
									else {
										problem("mpparked");
									}
									break;
				case	MPRAISING:	mpStatus.setText("Raising    "); 	
									if(ctrlState == CTRLST.RAISING) {
										// no change in controller state
										// no change in UI
										if(debug>0) System.out.println("mpraising");
									}
									else {
										problem("mpraising");
									}
									break;
				case	MPSTOPPED:	mpStatus.setText("Stopped    "); 	
									if(ctrlState == CTRLST.STOPPING || ctrlState == CTRLST.RAISING) {
										// update state
										ctrlState = CTRLST.STOPPED;
										// update UI
										ctrlStatus.setText("Stopped");
										orbitStart.setEnabled(true);
										orbitStop.setEnabled(false);
										orbitRaise.setEnabled(false);
										orbitLower.setEnabled(true);
										orbitHalt.setEnabled(false);
										if(debug>0) System.out.println("mpstopped");
									}		
									else {
										problem("mpstopped: current state = "+ctrlState);
									}
									break;
				case	MPSTARTING:	mpStatus.setText("Starting   "); 	
									if(ctrlState == CTRLST.STARTING) {
										// no change in controller state
										// update UI
										ctrlStatus.setText("Started");
										// no visibility changes required
										if(debug>0) System.out.println("mpstarting");
									}
									else {
										problem("mpstarting");
									}
									break;
				case	MPORBITING:	mpStatus.setText("Orbiting"); 			
									if(ctrlState == CTRLST.STARTING) {
										// state change
										ctrlState = CTRLST.ORBITING;
										// UI change
										ctrlStatus.setText("Orbiting");
										orbitStart.setEnabled(false);
										orbitStop.setEnabled(true);
										orbitRaise.setEnabled(false);
										orbitLower.setEnabled(false);
										if(debug>0) System.out.println("mporbiting");
									}
									else {
										problem("mporbiting");
									}
									break;
				case	MPSTOPPING:	mpStatus.setText("Stopping   "); 
									if(ctrlState == CTRLST.STOPPING) {
										// no change to controller state
										// no change in UI	
										if(debug>0) System.out.println("mpstopping");
									}
									else {
										problem("mpstopping");
									}
									break;
				case	MPLOWER:	mpStatus.setText("Lowering   "); 		
									// should only get here when controller has signaled lower
									if(ctrlState == CTRLST.LOWERING) {
										// no change in controller state
										// no change in UI
										if(debug>0) System.out.println("mplower");
									}
									else {
										problem("mplower");
									}
									break;
				case	MPHALT:		mpStatus.setText("Halted     ");
									if(ctrlState == CTRLST.HALT) {
										// no change in state
										ctrlStatus.setText("Halted"); 
										// no change in UI visibility
										if(debug>0) System.out.println("mphalt");
										finished = true;
									}	
									else {
										problem("mphalt");
									}
									break;
				default: 			mpStatus.setText("beats me"); 
									break;
			}	
		}
	}
	
	private void runCtrl() {
		// tell the MountPoints to start
		for(MountPoint mp : mtPts.values()) {
			mp.start();
		}
		
		// check button status and mount point statuses
		while(! connected) {
			// check our mount point states 
			refreshStates();
			
			// are we connected and locked as well?
			if(mpState == MPST.MPCONFG && ctrlState == CTRLST.CTRLCONF) {
				connected = true;
			}	
		}
		
		// send the configuration
		configMtPts(hag, mps, orb, ods, sst, tto);
		
		while(! finished) {
			// update the UI with friendly names
			refreshStates();		
		}
	}
	
	// problem
	// quick error handler 
	private static void problem(String problem) {
		System.err.println(problem);
		Delay.msDelay(10000);
	}
	// main
	public static void main(String[] args) {
		PCController2_2 ctrl1 = new PCController2_2();
		
		if(debug>0) System.out.println("starting.");
		
		// initialize - will start the UI
		ctrl1.initCtrl();
		
		// everything is set up - start running
		ctrl1.runCtrl();
		
	}

}
