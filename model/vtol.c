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
#define NY 2 // TODO: Get this right

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
    // Parameters
    // ********
   
    double *c_L_0, *c_L_alpha, *c_L_q, *c_L_delta_e; // Lift parameters
    c_L_0 = p[0];
    c_L_alpha = p[1];
    c_L_q = p[2];
    c_L_delta_e = p[3];

    double *M, *alpha_stall; // Blending function
    M = p[4];
    alpha_stall = p[5];

    double *c_D_p; // Drag parameters
    c_D_p = p[6];

    double *c_Y_beta, *c_Y_p, *c_Y_r, *c_Y_delta_a, *c_Y_delta_r; // Y-aerodynamic force
    c_Y_beta = p[7];
    c_Y_p = p[8];
    c_Y_r = p[9];
    c_Y_delta_a = p[10];
    c_Y_delta_r = p[11];

    double *c_l_beta, *c_l_p, *c_l_r, *c_l_delta_a, *c_l_delta_r; // Aerodynamic moment around x axis
    c_l_beta = p[12];
    c_l_p = p[13];
    c_l_r = p[13];
    c_l_delta_a = p[14];
    c_l_delta_r = p[15];

    double *c_m_0, *c_m_alpha, *c_m_q, *c_m_delta_e; // Aerodynamic moment around y axis
    c_m_0 = p[16];
    c_m_alpha = p[17];
    c_m_q = p[18];
    c_m_delta_e = p[19];

    double *c_n_beta, *c_n_p, *c_n_r, *c_n_delta_a, *c_n_delta_r; // Aerodynamic moment around z axis
    c_n_beta = p[20];
    c_n_p = p[21];
    c_n_r = p[22];
    c_n_delta_a = p[23];
    c_n_delta_r = p[24];

    // ********
    // Constants: Need to be set as constants in matlab script: nlgr.Parameters(i).Fixed = true;
    // ********

    // Propellers and motors
    double *rho; // Air density
    double *prop_diam_top, *prop_diam_pusher; // Propeller diameters
    double *c_F_top, *c_F_pusher; // Motor thrust constants
    double *c_Q_top, *c_Q_pusher; // Motor moment constants

    rho = p[25];
    prop_diam_top = p[26];
    prop_diam_pusher = p[27];
    c_F_top = p[28];
    c_F_pusher = p[29];
    c_Q_top = p[30];
    c_Q_pusher = p[31];

    // Airframe
    double *m; // Mass
    double *S; // Surface area
    double *chord; // Mean chord length
    double *b; // Wingspan
    double *lam; // -767 intermediate constants calculated from inertia matrix
    double *r_t1; // Position of motor 1
    double *r_t2; // Position of motor 2
    double *r_t3; // Position of motor 3
    double *r_t4; // Position of motor 4
    double *g; // Gravitational constant

    m = p[32];
    S = p[33];
    chord = p[34];
    b = p[35];
    lam = p[36]; // Vector of 9 elements
    r_t1 = p[37]; // Vector of 3 elements
    r_t2 = p[38]; // Vector of 3 elements
    r_t3 = p[39]; // Vector of 3 elements
    r_t4 = p[40]; // Vector of 3 elements
    g = p[41];

    // *******
    // State and input
    // *******
    // State: [v_body, ang_v_body, q_attitude, flap_deflection, flap_ang_vel]

    // v_body = u, v, w
    double vel_u, vel_v, vel_w;
    vel_u = x[0];
    vel_v = x[1];
    vel_w = x[2];

    // ang_v_body = p, q, r
    double ang_p, ang_q, ang_r;
    ang_p = x[3];
    ang_q = x[4];
    ang_r = x[5];

    // q_attitude = q0, q1, q2, q3
    double q0, q1, q2, q3;
    q0 = x[6];
    q1 = x[7];
    q2 = x[8];
    q3 = x[9];

    // Input: [n_t1, n_t2, n_t3, n_t4, n_p, delta_a_sp, delta_r_sp, delta_e_sp]
    double n_t1, n_t2, n_t3, n_t4, n_p;
    n_t1 = u[0];
    n_t2 = u[1];
    n_t3 = u[2];
    n_t4 = u[3];
    n_p = u[4];

    double delta_a_sp, delta_r_sp, delta_e_sp; // TODO: Replace with A-tail, instead of rudder and elevator?
    delta_a_sp = u[5];
    delta_e_sp = u[6];
    delta_r_sp = u[7];

    // ******
    // Forces 
    // ******

    // Calculate AoA and SSA, assuming no wind
    double V = sqrt(pow(vel_u,2) + pow(vel_v,2) + pow(vel_w,2));
    double alpha = atan(vel_w / vel_u);
    double beta = atan(vel_v / V);

    // Gravitational force
    double F_g[3];
    F_g[0] = 2 * ( q1 * q3 + q0 * q2);
    F_g[1] = 2 * (-q0 * q1 + q2 * q3);
    F_g[2] = pow(q0, 2) - pow(q1, 2) - pow(q2, 2) + pow(q3, 2);

    // Propeller forces
    double const_top_force = rho[0] * pow(prop_diam_top[0], 4); // TODO: This should not be computed at every iteration
    double const_pusher_force = rho[0] * pow(prop_diam_pusher[0], 4); // TODO: This should not be computed at every iteration
    double F_t1, F_t2, F_t3, F_t4, F_p;
    F_t1 = const_top_force * c_F_top[0] * pow(n_t1, 2);
    F_t2 = const_top_force * c_F_top[0] * pow(n_t2, 2);
    F_t3 = const_top_force * c_F_top[0] * pow(n_t3, 2);
    F_t4 = const_top_force * c_F_top[0] * pow(n_t4, 2);
    F_p = const_pusher_force * c_F_pusher[0] * pow(n_p, 2);

    double F_T[3];
    F_T[0] = F_p;
    F_T[1] = 0;
    F_T[2] = F_t1 + F_t2 + F_t3 + F_t4;

    // Aerodynamic forces
    double half_rho_S = 0.5 * rho[0] * S[0];
    // Blending function between linear lift model and flat plate lift model
    double delta = (1 + exp(-1 * M[0] * (alpha - alpha_stall[0])) + exp(M[0] * (alpha + alpha_stall[0])))
      / ((1 + exp(-1 * M[0] * (alpha - alpha_stall[0]))) * (1 + exp(M[0] * (alpha - alpha_stall[0]))));
    
    double sgn_alpha = alpha > 0 ? 1 : -1;
    double c_L_linear = c_L_0[0] + c_L_alpha[0] * alpha;
    double c_L_flat_plate = 2 * sgn_alpha * sin(alpha) * pow(sin(alpha), 2) * cos(alpha);
    double c_L = (1 - delta) * c_L_linear + delta * c_L_flat_plate;

    double F_lift = half_rho_S * pow(V, 2) * (
      c_L
      + c_L_q[0] * (chord[0] / (2 * V)) * ang_q
      + c_L_delta_e[0] * delta_e_sp
      );
    // TODO: Here we are assuming IMMEDIATE control surface response. This should be changed by adding control surface dynamics.

    double c_D = c_D_p[0] + pow(c_L_linear, 2);
    double F_drag = half_rho_S * pow(V, 2) * c_D;

    double F_aero[3];
    // Rotate from stability frame to body frame
    F_aero[0] = -cos(alpha) * F_drag + sin(alpha) * F_lift;
    F_aero[2] = -sin(alpha) * F_drag - cos(alpha) * F_lift;

    double c_Y = c_Y_beta[0] * beta
      + c_Y_p[0] * (b[0] / (2 * V)) * ang_p + c_Y_r[0] * (b[0] / (2 * V)) * ang_r
      + c_Y_delta_a[0] * delta_a_sp + c_Y_delta_r[0] * delta_r_sp;
    // TODO: Here we are assuming IMMEDIATE control surface response. This should be changed.
    F_aero[1] = half_rho_S * pow(V, 2) * c_Y;

    // Sum all forces
    double F_tot[3];
    F_tot[0] = F_g[0] + F_T[0] + F_aero[0];
    F_tot[1] = F_g[1] + F_T[1] + F_aero[1];
    F_tot[2] = F_g[2] + F_T[2] + F_aero[2];

    // ******
    // Moments
    // ******

    // Propeller moments
    double const_top_moment = rho[0] * pow(prop_diam_top[0], 5); // TODO: This should not be computed at every iteration
    double const_pusher_moment = rho[0] * pow(prop_diam_pusher[0], 5); // TODO: This should not be computed at every iteration
    double Q_t1, Q_t2, Q_t3, Q_t4, Q_p;
    Q_t1 = const_top_moment * c_Q_top[0] * pow(n_t1, 2);
    Q_t2 = const_top_moment * c_Q_top[0] * pow(n_t2, 2);
    Q_t3 = const_top_moment * c_Q_top[0] * pow(n_t3, 2);
    Q_t4 = const_top_moment * c_Q_top[0] * pow(n_t4, 2);
    //Q_p = const_pusher_moment * c_Q_pusher[0] * pow(n_p, 2); // TODO: Add if needed

    double Tau_Q[3];
    //Tau_Q[0] = Q_p; // TODO: Add propeller moment from pusher if needed
    Tau_Q[0] = 0;
    Tau_Q[1] = 0;
    Tau_Q[2] = - Q_t1 - Q_t2 + Q_t3 + Q_t4;

    // Gyroscopic moment
    // TODO: Add gyroscopic moment if needed
    // TODO: Get inertia of propellers if gyroscopic moment is needed

    // Propeller moments from thrust
    // Note that cross product is greatly reduced due to the propeller forces only acting along neg z-axis
    double Tau_T1[3], Tau_T2[3], Tau_T3[3], Tau_T4[3];
    Tau_T1[0] = - r_t1[1] * F_t1;
    Tau_T1[1] =   r_t1[0] * F_t1;
    Tau_T1[2] = 0;

    Tau_T2[0] = - r_t2[1] * F_t2;
    Tau_T2[2] =   r_t2[0] * F_t2;
    Tau_T2[2] = 0;

    Tau_T3[0] = - r_t3[1] * F_t3;
    Tau_T3[1] =   r_t3[0] * F_t3;
    Tau_T3[2] = 0;

    Tau_T4[0] = - r_t4[1] * F_t4;
    Tau_T4[1] =   r_t4[0] * F_t4;
    Tau_T4[2] = 0;

    double Tau_T[3];
    Tau_T[0] = Tau_T1[0] + Tau_T2[0] + Tau_T3[0] + Tau_T4[0];
    Tau_T[1] = Tau_T1[1] + Tau_T2[1] + Tau_T3[1] + Tau_T4[1];
    Tau_T[2] = Tau_T1[2] + Tau_T2[2] + Tau_T3[2] + Tau_T4[2];

    // Aerodynamic moments
    double c_l = c_l_beta[0] * beta
      + c_l_p[0] * (b[0] / (2 * V)) * ang_p
      + c_l_r[0] * (b[0] / (2 * V)) * ang_r
      + c_l_delta_a[0] * delta_a_sp
      + c_l_delta_r[0] * delta_r_sp;

    double c_m = c_m_0[0]
      + c_m_alpha[0] * alpha 
      + c_m_q[0] * ang_q
      + c_m_delta_e[0] * delta_e_sp; 

    double c_n = c_n_beta[0] * beta
      + c_n_p[0] * (b[0] / (2 * V)) * ang_p
      + c_n_r[0] * (b[0] / (2 * V)) * ang_r
      + c_n_delta_a[0] * delta_a_sp
      + c_n_delta_r[0] * delta_r_sp;

    double Tau_aero[3];
    Tau_aero[0] = half_rho_S * pow(V, 2) * b[0] * c_l;
    Tau_aero[1] = half_rho_S * pow(V, 2) * chord[0] * c_m;
    Tau_aero[2] = half_rho_S * pow(V, 2) * b[0] * c_n;

    // Sum all moments 
    double Tau_tot[3]; // TODO: Remember to add gyroscopic moment here if needed
    Tau_tot[0] = Tau_aero[0] + Tau_Q[0] + Tau_T[0];
    Tau_tot[1] = Tau_aero[1] + Tau_Q[1] + Tau_T[1];
    Tau_tot[2] = Tau_aero[2] + Tau_Q[2] + Tau_T[2];

    // *******
    // Dynamics
    // *******
    double vel_u_dot, vel_v_dot, vel_w_dot;
    vel_u_dot = (1/m[0]) * F_tot[0] - (ang_q*vel_w - ang_r*vel_v);
    vel_v_dot = (1/m[0]) * F_tot[1] - (ang_r*vel_u - ang_p*vel_w);
    vel_w_dot = (1/m[0]) * F_tot[2] - (ang_p*vel_v - ang_q*vel_u);

    dx[0] = vel_u_dot;
    dx[1] = vel_v_dot;
    dx[2] = vel_w_dot;

    double ang_p_dot, ang_q_dot, ang_r_dot;
    ang_p_dot = (lam[0] * ang_p * ang_q - lam[1] * ang_q * ang_r) + (lam[2] * Tau_tot[0] + lam[3] * Tau_tot[2]);
    ang_q_dot = (lam[4] * ang_p * ang_r - lam[5] * (ang_p * ang_p - ang_r * ang_r)) + ((1/lam[8]) * Tau_tot[1]);
    ang_r_dot = (lam[6] * ang_p * ang_q - lam[0] * ang_q * ang_r) + (lam[3] * Tau_tot[0] + lam[7] * Tau_tot[2]);

    dx[3] = ang_p_dot;
    dx[4] = ang_q_dot;
    dx[5] = ang_r_dot;

    double q0_dot, q1_dot, q2_dot, q3_dot;
    q0_dot = 0.5 * (-q1 * ang_p - q2 * ang_q - q3 * ang_r);
    q1_dot = 0.5 * ( q0 * ang_p - q3 * ang_q + q2 * ang_r);
    q2_dot = 0.5 * ( q3 * ang_p + q0 * ang_q - q1 * ang_r);
    q3_dot = 0.5 * (-q2 * ang_p + q1 * ang_q + q0 * ang_r);

    dx[6] = q0_dot;
    dx[7] = q1_dot;
    dx[8] = q2_dot;
    dx[9] = q3_dot;
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
    
    y[0] = x[0]; /* y[0]: Angular position. */
    y[1] = x[1]; /* y[1]: Angular velocity. */
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
