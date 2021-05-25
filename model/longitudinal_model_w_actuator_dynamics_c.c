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
    double *b; // Wingspan
    double *nondim_constant_lon; // Wingspan
    double *lam; // Intermediate constants calculated from inertia matrix
    double *J_yy; // Moment of inertia around y axis

    m = p[2];
    chord = p[3];
    b = p[4];
		nondim_constant_lon = p[5];
    lam = p[6]; // Vector of 8 elements
		J_yy = p[7];

		double *servo_time_const, *servo_rate_lim;
		servo_time_const = p[8];
		servo_rate_lim = p[9];

		double *elevator_trim;
		elevator_trim = p[10];

    // ********
    // Parameters
    // ********

    double *c_L_0, *c_L_alpha, *c_L_q, *c_L_delta_e; // Lift parameters
    c_L_0 = p[11];
    c_L_alpha = p[12];
    c_L_q = p[13];
    c_L_delta_e = p[14];

    double *c_D_p, *c_D_alpha, *c_D_alpha_sq, *c_D_q, *c_D_delta_e; // Drag parameters
    c_D_p = p[15];
    c_D_alpha = p[16];
    c_D_alpha_sq = p[17];
    c_D_q = p[18];
    c_D_delta_e = p[19];

    double *c_m_0, *c_m_alpha, *c_m_q, *c_m_delta_e; // Aerodynamic moment around y axis
    c_m_0 = p[20];
    c_m_alpha = p[21];
    c_m_q = p[22];
    c_m_delta_e = p[23];

    // *******
    // State and input
    // *******
    // State: [q_attitude, ang_v_body, vel_body]

    // q_attitude = q0, q1, q2, q3
    double q0, q1, q2, q3;
    q0 = x[0];
    q1 = x[1];
    q2 = x[2];
    q3 = x[3];

		// TODO: Consider normalizing q here!

    double ang_q;
    ang_q = x[4];

    // v_body = u, v, w
    double vel_u, vel_w;
    vel_u = x[5];
    vel_w = x[6];

		// Elevator dynamics
		double delta_e;
		delta_e = x[7];

    double delta_e_sp, n_t_fw;
    delta_e_sp = u[0] - elevator_trim[0];
    n_t_fw = u[1];

    // ******
    // Forces 
    // ******

    // Calculate AoA, assuming no wind
    double V = sqrt(pow(vel_u,2) + pow(vel_w,2));
    double alpha = atan(vel_w / vel_u);

    // Gravitational force
		double m_times_g = m[0] * g[0];
    double F_g[3];
    F_g[0] = 2 * ( q1 * q3 + q0 * q2) * m_times_g;
    F_g[1] = 2 * (-q0 * q1 + q2 * q3) * m_times_g;
    F_g[2] = (pow(q0, 2) - pow(q1, 2) - pow(q2, 2) + pow(q3, 2)) * m_times_g;

    // Aerodynamic forces
    double c_L = c_L_0[0] + c_L_alpha[0] * alpha
      + c_L_q[0] * (nondim_constant_lon[0]) * ang_q
      + c_L_delta_e[0] * delta_e;
    double F_lift = half_rho_planform[0] * pow(V, 2) * c_L;

    double c_D = c_D_p[0] + c_D_alpha[0] * alpha + c_D_alpha_sq[0] * pow(alpha, 2)
      + c_D_q[0] * (nondim_constant_lon[0]) * ang_q
      + c_D_delta_e[0] * delta_e;
    double F_drag = half_rho_planform[0] * pow(V, 2) * c_D;

    double F_aero[3];

    // Rotate from stability frame to body frame
    F_aero[0] = -cos(alpha) * F_drag + sin(alpha) * F_lift;
    F_aero[2] = -sin(alpha) * F_drag - cos(alpha) * F_lift;

    // Sum all forces
    double F_tot[3];
    F_tot[0] = F_g[0] + F_aero[0];
    F_tot[2] = F_g[2] + F_aero[2];

    // ******
    // Moments
    // ******

    // Aerodynamic moments
    double c_m = c_m_0[0]
      + c_m_alpha[0] * alpha
      + c_m_q[0] * nondim_constant_lon[0] * ang_q
      + c_m_delta_e[0] * delta_e;

    double m_moment;
    m_moment = half_rho_planform[0] * pow(V, 2) * chord[0] * c_m;

    // *******
    // Dynamics
    // *******
    double q0_dot, q1_dot, q2_dot, q3_dot;
    q0_dot = - 0.5 * q2 * ang_q;
    q1_dot = - 0.5 * q3 * ang_q;
    q2_dot = 0.5 * q0 * ang_q;
    q3_dot = 0.5 * q1 * ang_q;

    dx[0] = q0_dot;
    dx[1] = q1_dot;
    dx[2] = q2_dot;
    dx[3] = q3_dot;

    double ang_q_dot;
    ang_q_dot = (1 / J_yy[0]) * m_moment;

    dx[4] = ang_q_dot;

    double vel_u_dot, vel_w_dot;
    vel_u_dot = (1/m[0]) * F_tot[0] - ang_q * vel_w;
    vel_w_dot = (1/m[0]) * F_tot[2] + ang_q * vel_u;

    dx[5] = vel_u_dot;
    dx[6] = vel_w_dot;

		// NOTE: Elevator is not an output, as it is not measured
		double delta_e_dot;
		delta_e_dot = bound(
				-1 / servo_time_const[0] * delta_e + 1 / servo_time_const[0] * delta_e_sp,
				-servo_rate_lim[0],
				servo_rate_lim[0]);
    dx[7] = delta_e_dot;
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
