/*   Copyright 2005-2006 The MathWorks, Inc. */

/* Template file for IDNLGREY model specification.
   
   Use this file to create a MEX function that specifies the model
   structure and equations. The MEX file syntax is
      [dx, y] = mymodel(t, x, u, p1, p2, ..., pn, auxvar)
   where
      * t is the time (scalar).
      * x is the state vector at time t (column vector).
      * u is the vector of inputs at time t (column vector).
      * p1, p2,... pn: values of the estimated parameters specified
        in the IDNLGREY model.
      * auxvar: a cell array containing auxiliary data in any format
        (optional).
      * dx is the vector of state derivatives at time t (column vector).
      * y is the vector of outputs at time t.
   
   To create the MEX file "mymodel", do the following:
      1) Save this template as "mymodel.c" (replace "mymodel" by the
         name of your choice).
      2) Define the number NY of outputs below.
      3) Specify the state derivative equations in COMPUTE_DX below.
      4) Specify the output equations in COMPUTE_Y below.
      5) Build the MEX file using
            >> mex mymodel.c
*/

/* Include libraries. */
#include "mex.h"
#include "math.h"

/* Specify the number of outputs here. */
#define NY 7
// Taken from this page: https://stackoverflow.com/questions/3437404/min-and-max-in-c
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
#define bound(x,bl,bu) (min(max(x,bl),bu))

/* State equations. */
void compute_dx(
    double *dx,  /* Vector of state derivatives (length nx). */
    double t,    /* Time t (scalar). */
    double *x,   /* State vector (length nx). */
    double *u,   /* Input vector (length nu). */
    double **p,  /* p[j] points to the j-th estimated model parameters (a double array). */
    const mxArray *auxvar  /* Cell array of additional data. */
   )
{
    /*
      Define the state equation dx = f(t, x, u, p[0],..., p[np-1], auvar)
      in the body of this function.
    */

    // ********
    // Constants
    // ********

    double *g;
		double *half_rho_planform;

		g = p[0];
		half_rho_planform = p[1];

    // Airframe
    double *m; // Mass
    double *chord; // Mean chord length
    double *wingspan;
    double *nondim_constant_lon;
    double *nondim_constant_lat;
    double *lam; // Intermediate constants calculated from inertia matrix
    double *J_yy; // Moment of inertia around y axis

    m = p[2];
    chord = p[3];
    wingspan = p[4];
		nondim_constant_lon = p[5];
		nondim_constant_lat = p[6];
    lam = p[7]; // Vector of 8 elements
		J_yy = p[8];

		double lam_1, lam_2, lam_3, lam_4, lam_5, lam_6, lam_7, lam_8;
		lam_1 = lam[0];
		lam_2 = lam[1];
		lam_3 = lam[2];
		lam_4 = lam[3];
		lam_5 = lam[4];
		lam_6 = lam[5];
		lam_7 = lam[6];
		lam_8 = lam[7];

		double *servo_time_const, *servo_rate_lim;
		servo_time_const = p[9];
		servo_rate_lim = p[10];

		double *aileron_trim, *elevator_trim, *rudder_trim;
		aileron_trim = p[11];
		elevator_trim = p[12];
		rudder_trim = p[13];

    // ********
    // Parameters
    // ********

		// Y-aerodynamic force
    double *c_Y_p, *c_Y_r, *c_Y_delta_a, *c_Y_delta_r;
    c_Y_p = p[14];
    c_Y_r = p[15];
    c_Y_delta_a = p[16];
    c_Y_delta_r = p[17];

		// Moment around x-axis
    double *c_l_p, *c_l_r, *c_l_delta_a, *c_l_delta_r;
    c_l_p = p[18];
    c_l_r = p[19];
    c_l_delta_a = p[20];
    c_l_delta_r = p[21];

		// Moment around z-axis
    double *c_n_p, *c_n_r, *c_n_delta_a, *c_n_delta_r;
    c_n_p = p[22];
    c_n_r = p[23];
    c_n_delta_a = p[24];
    c_n_delta_r = p[25];


    // *******
    // State and input
    // *******
    // State: [q_attitude, ang_v_body, vel_body]

    // q_attitude = e0, e1, e2, e3
    double e0, e1, e2, e3;
    e0 = x[0];
    e1 = x[1];
    e2 = x[2];
    e3 = x[3];

		// Make sure quaternion remains a unit quaternion
		double quat_norm = sqrt(pow(e0,2) + pow(e1,2) + pow(e2,2) + pow(e3,2));
		e0 = e0 / quat_norm;
		e1 = e1 / quat_norm;
		e2 = e2 / quat_norm;
		e3 = e3 / quat_norm;

    double ang_p, ang_r;
    ang_p = x[4];
    ang_r = x[5];

    // v_body = u, v, w
    double vel_v;
    vel_v = x[6];

		// Elevator dynamics
		double delta_a, delta_r;
		delta_a = x[7];
		delta_r = x[8];

    double delta_a_sp, delta_r_sp;
    delta_a_sp = u[0] - aileron_trim[0];
    delta_r_sp = u[1] - rudder_trim[0];

		double vel_u, vel_w;
		// Treat u and w as pure inputs, as these are needed for the airspeed but not modelled
		vel_u = u[2];
		vel_w = u[3];

    // ******
    // Forces 
    // ******

    double V = sqrt(pow(vel_u,2) + pow(vel_v,2) + pow(vel_w,2));

    // Gravitational force
		double m_times_g = m[0] * g[0];
    double f_G_y = m_times_g * (-e0 * e1 + e2 * e3);

    // Aerodynamic forces
    double c_Y = c_Y_p[0] * nondim_constant_lat[0] * ang_p
			+ c_Y_r[0] * nondim_constant_lat[0] * ang_r
			+ c_Y_delta_a[0] * delta_a
			+ c_Y_delta_r[0] * delta_r;

    double f_Y = half_rho_planform[0] * pow(V, 2) * wingspan[0] * c_Y;

    // Sum all forces
    double F_tot_y;
    F_tot_y = f_G_y + f_Y;

    // ******
    // Moments
    // ******

    // Aerodynamic moments
    double c_l = c_l_p[0] * nondim_constant_lat[0] * ang_p
			+ c_l_r[0] * nondim_constant_lat[0] * ang_r
			+ c_l_delta_a[0] * delta_a
			+ c_l_delta_r[0] * delta_r;

    double c_n = c_n_p[0] * nondim_constant_lat[0] * ang_p
			+ c_n_r[0] * nondim_constant_lat[0] * ang_r
			+ c_n_delta_a[0] * delta_a
			+ c_n_delta_r[0] * delta_r;

    double tau_x, tau_z;
    tau_x = half_rho_planform[0] * pow(V, 2) * wingspan[0] * c_l;
    tau_z = half_rho_planform[0] * pow(V, 2) * wingspan[0] * c_n;

    // *******
    // Kinematics 
    // *******

    double e0_dot, e1_dot, e2_dot, e3_dot;
    e0_dot = 0.5 * (- e1 * ang_p - e3 * ang_r);
    e1_dot = 0.5 * (e0 * ang_p + e2 * ang_r);
    e2_dot = 0.5 * (e3 * ang_p - e1 * ang_r);
    e3_dot = 0.5 * (- e2 * ang_p + e0 * ang_r);

    dx[0] = e0_dot;
    dx[1] = e1_dot;
    dx[2] = e2_dot;
    dx[3] = e3_dot;

    // *******
    // Dynamics
    // *******

    double ang_p_dot, ang_r_dot;
    ang_p_dot = lam_3 * tau_x + lam_4 * tau_z;
    ang_r_dot = lam_4 * tau_x + lam_8 * tau_z;

    dx[4] = ang_p_dot;
    dx[5] = ang_r_dot;

    double vel_v_dot;
    vel_v_dot = (1/m[0]) * F_tot_y;

    dx[6] = vel_v_dot;

		// Model control surfaces as rate limited first order responses
		double delta_a_dot, delta_r_dot;
		delta_a_dot = bound(
				-1 / servo_time_const[0] * delta_a + 1 / servo_time_const[0] * delta_a_sp,
				-servo_rate_lim[0],
				servo_rate_lim[0]);

		delta_r_dot = bound(
				-1 / servo_time_const[0] * delta_r + 1 / servo_time_const[0] * delta_r_sp,
				-servo_rate_lim[0],
				servo_rate_lim[0]);

    dx[7] = delta_a_dot;
    dx[8] = delta_r_dot;
}

/* Output equations. */
void compute_y(
    double *y,   /* Vector of outputs (length NY). */
    double t,    /* Time t (scalar). */
    double *x,   /* State vector (length nx). */
    double *u,   /* Input vector (length nu). */
    double **p,  /* p[j] points to the j-th estimated model parameters (a double array). */
    const mxArray *auxvar  /* Cell array of additional data. */
   )
{
    /*
      Define the output equation y = h(t, x, u, p[0],..., p[np-1], auvar)
      in the body of this function.
    */

    /*
      Accessing the contents of auxvar: see the discussion in compute_dx.
    */

    /* Example code from ODE function for DCMOTOR example
      used in idnlgreydemo1 (dcmotor_c.c) follows.
    */

    y[0] = x[0];
    y[1] = x[1];
    y[2] = x[2];
    y[3] = x[3];
    y[4] = x[4];
    y[5] = x[5];
    y[6] = x[6];
		// Control surface deflections are not outputs, as these are not measured
}




/*----------------------------------------------------------------------- *
   DO NOT MODIFY THE CODE BELOW UNLESS YOU NEED TO PASS ADDITIONAL
   INFORMATION TO COMPUTE_DX AND COMPUTE_Y
 
   To add extra arguments to compute_dx and compute_y (e.g., size
   information), modify the definitions above and calls below.
 *-----------------------------------------------------------------------*/

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    /* Declaration of input and output arguments. */
    double *x, *u, **p, *dx, *y, *t;
    int     i, np, nu, nx;
    const mxArray *auxvar = NULL; /* Cell array of additional data. */
    
    if (nrhs < 3) {
        mexErrMsgIdAndTxt("IDNLGREY:ODE_FILE:InvalidSyntax",
        "At least 3 inputs expected (t, u, x).");
    }
    
    /* Determine if auxiliary variables were passed as last input.  */
    if ((nrhs > 3) && (mxIsCell(prhs[nrhs-1]))) {
        /* Auxiliary variables were passed as input. */
        auxvar = prhs[nrhs-1];
        np = nrhs - 4; /* Number of parameters (could be 0). */
    } else {
        /* Auxiliary variables were not passed. */
        np = nrhs - 3; /* Number of parameters. */
    }
    
    /* Determine number of inputs and states. */
    nx = mxGetNumberOfElements(prhs[1]); /* Number of states. */
    nu = mxGetNumberOfElements(prhs[2]); /* Number of inputs. */
    
    /* Obtain double data pointers from mxArrays. */
    t = mxGetPr(prhs[0]);  /* Current time value (scalar). */
    x = mxGetPr(prhs[1]);  /* States at time t. */
    u = mxGetPr(prhs[2]);  /* Inputs at time t. */
    
    p = mxCalloc(np, sizeof(double*));
    for (i = 0; i < np; i++) {
        p[i] = mxGetPr(prhs[3+i]); /* Parameter arrays. */
    }
    
    /* Create matrix for the return arguments. */
    plhs[0] = mxCreateDoubleMatrix(nx, 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(NY, 1, mxREAL);
    dx      = mxGetPr(plhs[0]); /* State derivative values. */
    y       = mxGetPr(plhs[1]); /* Output values. */
    
    /*
      Call the state and output update functions.
      
      Note: You may also pass other inputs that you might need,
      such as number of states (nx) and number of parameters (np).
      You may also omit unused inputs (such as auxvar).
      
      For example, you may want to use orders nx and nu, but not time (t)
      or auxiliary data (auxvar). You may write these functions as:
          compute_dx(dx, nx, nu, x, u, p);
          compute_y(y, nx, nu, x, u, p);
    */
    
    /* Call function for state derivative update. */
    compute_dx(dx, t[0], x, u, p, auxvar);
    
    /* Call function for output update. */
    compute_y(y, t[0], x, u, p, auxvar);
    
    /* Clean up. */
    mxFree(p);
}
