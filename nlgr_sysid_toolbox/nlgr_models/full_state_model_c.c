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
#define NY 10
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

		// Longitudinal parameters

    double *c_L_0, *c_L_alpha, *c_L_q, *c_L_delta_e; // Lift parameters
    c_L_0 = p[14];
    c_L_alpha = p[15];
    c_L_q = p[16];
    c_L_delta_e = p[17];

    double *c_D_p, *c_D_alpha, *c_D_alpha_sq, *c_D_q, *c_D_delta_e; // Drag parameters
    c_D_p = p[18];
    c_D_alpha = p[19];
    c_D_alpha_sq = p[20];
    c_D_q = p[21];
    c_D_delta_e = p[22];

    double *c_m_0, *c_m_alpha, *c_m_q, *c_m_delta_e; // Aerodynamic moment around y axis
    c_m_0 = p[23];
    c_m_alpha = p[24];
    c_m_q = p[25];
    c_m_delta_e = p[26];

		// Lateral parameters

		// Y-aerodynamic force
    double *c_Y_beta, *c_Y_p, *c_Y_r, *c_Y_delta_a, *c_Y_delta_r;
    c_Y_beta = p[27];
    c_Y_p = p[28];
    c_Y_r = p[29];
    c_Y_delta_a = p[30];
    c_Y_delta_r = p[31];

		// Moment around x-axis
    double *c_l_beta, *c_l_p, *c_l_r, *c_l_delta_a, *c_l_delta_r;
    c_l_beta = p[32];
    c_l_p = p[33];
    c_l_r = p[34];
    c_l_delta_a = p[35];
    c_l_delta_r = p[36];

		// Moment around z-axis
    double *c_n_beta, *c_n_p, *c_n_r, *c_n_delta_a, *c_n_delta_r;
    c_n_beta = p[37];
    c_n_p = p[38];
    c_n_r = p[39];
    c_n_delta_a = p[40];
    c_n_delta_r = p[41];

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

    double ang_p, ang_q, ang_r;
    ang_p = x[4];
    ang_q = x[5];
    ang_r = x[6];

    // v_body = u, v, w
    double vel_u, vel_v, vel_w;
    vel_u = x[7];
    vel_v = x[8];
    vel_w = x[9];

		// Elevator dynamics
		double delta_a, delta_e, delta_r;
		delta_a = x[10];
		delta_e = x[11];
		delta_r = x[12];

    double delta_a_sp, delta_e_sp, delta_r_sp;
    delta_a_sp = u[4] - aileron_trim[0];
    delta_e_sp = u[5] - elevator_trim[0];
    delta_r_sp = u[6] - rudder_trim[0];

    // ******
    // Forces
    // ******

    double V = sqrt(pow(vel_u,2) + pow(vel_v,2) + pow(vel_w,2));
    double alpha = atan2(vel_w, vel_u);
		double beta = asin(vel_v / V);

    // Gravitational force
		double m_times_g = m[0] * g[0];
    double F_g_x, F_g_y, F_g_z;

    F_g_x = 2 * m_times_g * (e1 * e3 - e0 * e2);
    F_g_y = 2 * m_times_g * (e2 * e3 + e0 * e1);
    F_g_z = m_times_g * (pow(e0, 2) - pow(e1, 2) - pow(e2, 2) + pow(e3, 2));

    // Aerodynamic forces
    double c_L = c_L_0[0] + c_L_alpha[0] * alpha
      + c_L_q[0] * (nondim_constant_lon[0]) * ang_q
      + c_L_delta_e[0] * delta_e;

    double c_Y = beta * c_Y_beta[0]
			+ c_Y_p[0] * nondim_constant_lat[0] * ang_p
			+ c_Y_r[0] * nondim_constant_lat[0] * ang_r
			+ c_Y_delta_a[0] * delta_a
			+ c_Y_delta_r[0] * delta_r;

    double c_D = c_D_p[0] + c_D_alpha[0] * alpha + c_D_alpha_sq[0] * pow(alpha, 2)
      + c_D_q[0] * (nondim_constant_lon[0]) * ang_q
      + c_D_delta_e[0] * delta_e;

    double F_lift = half_rho_planform[0] * pow(V, 2) * c_L;
    double F_drag = half_rho_planform[0] * pow(V, 2) * c_D;

    double X, Y, Z;

    // Rotate from stability frame to body frame
    X = -cos(alpha) * F_drag + sin(alpha) * F_lift;
    Y = half_rho_planform[0] * pow(V, 2) * wingspan[0] * c_Y;
    // Rotate from stability frame to body frame
    Z = -sin(alpha) * F_drag - cos(alpha) * F_lift;

    // Sum all forces
    double F_tot_x, F_tot_y, F_tot_z;
    F_tot_x = F_g_x + X;
    F_tot_y = F_g_y + Y;
    F_tot_z = F_g_z + Z;

    // ******
    // Moments
    // ******

    // Aerodynamic moments
    double c_l = beta * c_l_beta[0]
			+ c_l_p[0] * nondim_constant_lat[0] * ang_p
			+ c_l_r[0] * nondim_constant_lat[0] * ang_r
			+ c_l_delta_a[0] * delta_a
			+ c_l_delta_r[0] * delta_r;

    double c_m = c_m_0[0]
      + c_m_alpha[0] * alpha
      + c_m_q[0] * nondim_constant_lon[0] * ang_q
      + c_m_delta_e[0] * delta_e;

    double c_n = beta * c_n_beta[0]
			+ c_n_p[0] * nondim_constant_lat[0] * ang_p
			+ c_n_r[0] * nondim_constant_lat[0] * ang_r
			+ c_n_delta_a[0] * delta_a
			+ c_n_delta_r[0] * delta_r;

    double L, M, N;
    L = half_rho_planform[0] * pow(V, 2) * wingspan[0] * c_l;
    M = half_rho_planform[0] * pow(V, 2) * chord[0] * c_m;
    N = half_rho_planform[0] * pow(V, 2) * wingspan[0] * c_n;

    // *******
    // Kinematics
    // *******

    double e0_dot, e1_dot, e2_dot, e3_dot;
    e0_dot = 0.5 * (- e1 * ang_p - e2 * ang_q - e3 * ang_r);
    e1_dot = 0.5 * (e0 * ang_p - e3 * ang_q + e2 * ang_r);
    e2_dot = 0.5 * (e3 * ang_p + e0 * ang_q - e1 * ang_r);
    e3_dot = 0.5 * (- e2 * ang_p + e1 * ang_q + e0 * ang_r);

    dx[0] = e0_dot;
    dx[1] = e1_dot;
    dx[2] = e2_dot;
    dx[3] = e3_dot;

    // *******
    // Dynamics
    // *******

    double ang_p_dot, ang_q_dot, ang_r_dot;
    ang_p_dot = lam_1 * ang_p * ang_q - lam_2 * ang_q * ang_r
			+ lam_3 * L + lam_4 * N;
    ang_q_dot = lam_5 * ang_p * ang_r - lam_6 * (pow(ang_p, 2) - pow(ang_r, 2))
			+ (1 / J_yy[0]) * M;
    ang_r_dot = lam_7 * ang_p * ang_q - lam_1 * ang_q * ang_r
			+ lam_4 * L + lam_8 * N;

    dx[4] = ang_p_dot;
    dx[5] = ang_q_dot;
    dx[6] = ang_r_dot;

    double vel_u_dot, vel_v_dot, vel_w_dot;
    vel_u_dot = ang_r * vel_v - ang_q * vel_w + (1/m[0]) * F_tot_x;
    vel_v_dot = ang_p * vel_w - ang_r * vel_u + (1/m[0]) * F_tot_y;
    vel_w_dot = ang_q * vel_u - ang_p * vel_v + (1/m[0]) * F_tot_z;

    dx[7] = vel_u_dot;
    dx[8] = vel_v_dot;
    dx[9] = vel_w_dot;

		// Model control surfaces as rate limited first order responses
		double delta_a_dot, delta_e_dot, delta_r_dot;

		delta_a_dot = bound(
				-1 / servo_time_const[0] * delta_a + 1 / servo_time_const[0] * delta_a_sp,
				-servo_rate_lim[0],
				servo_rate_lim[0]);

		delta_e_dot = bound(
				-1 / servo_time_const[0] * delta_e + 1 / servo_time_const[0] * delta_e_sp,
				-servo_rate_lim[0],
				servo_rate_lim[0]);

		delta_r_dot = bound(
				-1 / servo_time_const[0] * delta_r + 1 / servo_time_const[0] * delta_r_sp,
				-servo_rate_lim[0],
				servo_rate_lim[0]);

    dx[10] = delta_a_dot;
    dx[11] = delta_e_dot;
    dx[12] = delta_r_dot;
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
    y[7] = x[7];
    y[8] = x[8];
    y[9] = x[9];
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