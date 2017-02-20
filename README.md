# KiteX PX4 Firmware #

## Main modifications ##

1. Created a new VTOL type (kite) with custom transition mixing of MC and FW controls.
2. Modified FW attitude controller to follow a predefined path.
3. Modified MC attitude controller to not fight yaw
4. Modified MC position controller to have a constant pitch for tether tension.  
5. Created new airframe 13013 KiteX
6. Created custom mixer vtol_kitex


## How to fly/use ##

1. Take off in manual mode
2. Fly to the kite to the right side of the centre of looping.
3. Set the throttle to between 80-100%
4. Transition to FW flight mode.
⋅⋅* Looping
5. Transtion to MC flight mode when the kite is flying upwards.
6. Land.

## Parameters that needs to be set ##

**Pos_B** // position of tether anchoring point in the local coordinate system (if kite is turned on at the origin pos_b = 0).

PARAM_DEFINE_FLOAT(MPC_X_POS_B, 0.0f);
PARAM_DEFINE_FLOAT(MPC_Y_POS_B, 0.0f);
PARAM_DEFINE_FLOAT(MPC_Z_POS_B, 0.0f);


**Angles to C.** Defines the centre of looping relative to the anchoring point.

Phi is relative to the X-axis, theta to the XY-plane, positive for negative z.

PARAM_DEFINE_FLOAT(MPC_PHI_C, 0.0f);
PARAM_DEFINE_FLOAT(MPC_THETA_C, 0.0f);

**turn_r** Turning radius in the Pi-plane (general turning radius - 23 meters could be a starting value)

PARAM_DEFINE_FLOAT(MPC_LOOP_TURN_R, 0.0f);


## “manual control” ##
We make use of the manual remote aux channels. To control various failsafe methods and enable tethered hovering mode while in MC flight mode. AUX


### Aux1 ###  
Activated above 0.0f.
- Disables yaw compensation **(should be high for the entire tethered flight).**
- Enables fixed pitch in MC mode (not velocity controlled mode only => manual and altitude mode)
Files: mc_att_control, mc_pos_control

### Aux2 ###
AUX2 > 0.0f activates is a failsafe that’s intented to protect against crashing into the ground.
Files: vtol_att_control/kite.cpp

### Aux3 ###
AUX3 > 0.0f turns on an automatic failsafe if the kites is below 10 meters above ground.
Files: vtol_att_control/kite.cpp


## Other information ##

### Frame of reference ###

The orientation of the local reference frame is like you would expect from a multicopter.

While hovering at the time of takeoff.
1. Positive X is in the direction from the tether anchoring point to the drone.
1. Positive Y is to the right looking from the tether anchoring point towards the drone along the span of the main wing.
1. Positive Z is downwards towards the centre of earth.

The frame of reference doesn't chance for the FW flight mode hence what's normally considered yaw for a fixed wing (FW) aircraft is roll in the local reference frame ect.

### VTOL TYPE ###

VTOL type = 1 is the kite

```
enum vtol_type {
	TAILSITTER = 0,
	KITE,
	TILTROTOR,
	STANDARD
};
```

### Other parameters ###

**Param MPC_TET_POS_CTL**
Activates above 0.5
// not current in use. Can be used for more sophisticated hovering on the sphere. In position control (using offboard or loiter). Be careful if in “manual” position mode as position setup is not forced to be on the sphere - only the velocity set point is set to be tangential.

PARAM_DEFINE_FLOAT(MPC_TET_POS_CTL, 0);


## KiteX SuperQ kite specification ##

The source code has been tested with two KiteX SuperQ demonstration models. For software in the loop simulation the following parameters can be used.

#### Inertial properties (without tether) ####
* mass: 1.4 kg
* J: the following matrix has been used for previous simulations.

```
function generateJ() {
   // Moment of Inertia kite
    var JkiteX = 0.15 kg m2
    var JkiteY = 0.05 kg m2
    var JkiteZ = 0.15 kg m2

    var J = new Matrix3()
    // J.set( 11, 12, 13,
    //        21, 22, 23,
    //        31, 32, 33 );
    return J.set( JkiteX, 0, 0,
          0, JkiteY, 0,
          0, 0, JkiteZ );
}
```

#### Main Wing: ####
* airfoil: asymmetric low Reynolds number single airfoil
* cord: 140 mm
* span: 1400 mm
* angle of incidence: 5 deg negative rotation about Y
* distance to COG: (x,y,z): (0,0,0)

#### Vertical wing: ####
* airfoil: asymmetric low Reynolds number single element airfoil
* cord: 140 mm
* span: 600 (x2) mm
* angle of incidence: 8 deg positive rotation about X
* distance to COG: (x,y,z): (0,0,0)

#### Rudder ####
* span: 600 mm
* cord: 60 mm
* airfoil: NACA 0012
* distance to COG: (x,y,z): (0,0,700)
* movement: ~ +- 30 deg with the 8000,8000 PX4 mixer setting.

#### Elevator ####
* span: 600 mm
* cord: 60 mm
* airfoil: NACA 0012
* distance to COG: (x,y,z): (0,0,630)
* movement: ~+- 40 deg. Neutral at ~ 24 degree positive Y
* note: Should be parallel to fuselage at a normalised output of 0.6.

#### Tether ####
* length: 80 m
* bridle attachment points on kite (3): (x,y,z): (0, (-680, 0, 680), 0)
* brindle connection point: (x,y,z): (-2400, 0, 0)
* mass: ~ 0.3 kg evenly distributed
* thickness: ~ 1.5 mm

#### Rotors ####
* model: DAL T5045C Cyclone
* motor: Emax RS2205S 2300KV "Red Bottom" RaceSpec Motor
* Bench test:
[miniQuadtestbench](https://www.miniquadtestbench.com/2300kv-shootout-emax-rs2205-2300kv.html)
[rceagle](https://www.rceagle.com/blog/thrust-test-emxax-rs2205-2300kv-motor-mit-verschieden-dal-props-propellern-und-3s-4s-lipo)
* maximum static thrust: ~617g ~= 7N
* voltage: 3s lipo ~ 11.1 V
* Max RPM: ~ 22.000 RPM
