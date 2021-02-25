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

    // *******
    // Retrieve state variables
    // *******
    // State: [v_body, ang_v_body, q_attitude, flap_deflection, flap_ang_vel]
    // v_body = u, v, w
    double u, v, w;
    u = x[0];
    v = x[1];
    w = x[2];

    // ang_v_body = p, q, r
    double p, q, r;
    p = x[3];
    q = x[4];
    r = x[5];

    // q_attitude = q0, q1, q2, q3
    double q0, q1, q2, q3;
    q0 = x[6];
    q1 = x[7];
    q2 = x[8];
    q3 = x[9];

    // *******
    // Dynamics
    // *******
    double u_dot, v_dot, w_dot;
    u_dot = (1/m) * F_tot[0] - (q*w - r*v);
    v_dot = (1/m) * F_tot[1] - (r*u - p*w);
    w_dot = (1/m) * F_tot[2] - (p*v - q*u);

    dx[0] = u_dot;
    dx[1] = v_dot;
    dx[2] = w_dot;

    // TODO: Compute lam constants
    // Lam constants defined from inertia matrix to reduce computations
    double p_dot, q_dot, r_dot;
    p_dot = (lam1 * p * q - lam2 * q * r) + (lam3 * Tau_tot[0] + lam4 * Tau_tot[2]);
    q_dot = (lam5 * p * r - lam6 * (p * p - r * r)) + ((1/Jy) * Tau_tot[1]);
    r_dot = (lam7 * p * q - lam1 * q * r) + (lam4 * Tau_tot[0] + lam8 * Tau_tot[2]);

    dx[3] = p_dot;
    dx[4] = q_dot;
    dx[5] = r_dot;

    double q0_dot, q1_dot, q2_dot, q3_dot;
    q0_dot = 0.5 * (-q1 * p - q2 * q - q3 * r);
    q1_dot = 0.5 * ( q0 * p - q3 * q + q2 * r);
    q2_dot = 0.5 * ( q3 * p + q0 * q - q1 * r);
    q3_dot = 0.5 * (-q2 * p + q1 * q + q0 * r);

    dx[6] = q0_dot;
    dx[7] = q1_dot;
    dx[8] = q2_dot;
    dx[9] = q3_dot;

    // NOTE: From template: How to extract parameters
    double *tau, *k; /* Estimated model parameters. */
    tau = p[0];
    k   = p[1];
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
