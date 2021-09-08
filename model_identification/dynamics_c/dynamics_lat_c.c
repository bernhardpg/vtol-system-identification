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

double interp(double x0, double x1, double y0, double y1, double xp)
{
    double yp = y0 + ((y1 - y0) / (x1 - x0)) * (xp - x0);
    return yp;
}

void compute_u_at_t(double t, double *u, double *u_at_t, int nu_rows, int nu_cols)
{
    int interp_end_index = 1;
    double curr_t = u[interp_end_index];
    // Move index to the location where we are at the closest next time step
    // in the input sequence
    while ((curr_t < t) && (interp_end_index < nu_rows - 1))
    {
        interp_end_index++;
        curr_t = u[interp_end_index]; // First column in u is timestamp
    }

    for (int col_i = 0; col_i < nu_cols - 1; ++col_i)
    {
        double y0 = u[interp_end_index - 1 + nu_rows * (col_i + 1)];
        double y1 = u[interp_end_index + nu_rows * (col_i + 1)];

        u_at_t[col_i] = interp(u[interp_end_index - 1], u[interp_end_index], y0, y1, t);
    }
}

/* State equations. */
void compute_dx(
    double *dx,  /* Vector of state derivatives (length nx). */
    double t,    /* Time t (scalar). */
    double *x,   /* State vector (length nx). */
    double *u,   /* Input vector (length nu). */
    double *p,  /* p[j] points to the j-th estimated model parameters (a double array). */
		int nu_rows,
		int nu_cols
   )
{
    /*
      Define the state equation dx = f(t, x, u, p[0],..., p[np-1], auvar)
      in the body of this function.
    */

    // Extract parameters
    double rho, mass_kg, g, wingspan_m, mean_aerodynamic_chord_m, planform_sqm, V_nom,
        gam_1, gam_2, gam_3, gam_4, gam_5, gam_6, gam_7, gam_8, J_yy;

    rho = p[0];
    mass_kg = p[1];
    g = p[2];
    wingspan_m = p[3];
    mean_aerodynamic_chord_m = p[4];
    planform_sqm = p[5];
    V_nom = p[6];
    gam_1 = p[7];
    gam_2 = p[8];
    gam_3 = p[9];
    gam_4 = p[10];
    gam_5 = p[11];
    gam_6 = p[12];
    gam_7 = p[13];
    gam_8 = p[14];
    J_yy = p[15];

    double c_Y_0, c_Y_v, c_Y_p, c_Y_r, c_Y_delta_a, c_Y_delta_r,
           c_l_0, c_l_v, c_l_p, c_l_r, c_l_delta_a, c_l_delta_r,
           c_n_0, c_n_v, c_n_p, c_n_r, c_n_delta_a, c_n_delta_r;

    c_Y_0 = p[16];
    c_Y_v = p[17];
    c_Y_p = p[18];
    c_Y_r = p[19];
    c_Y_delta_a = p[20];
    c_Y_delta_r = p[21];

    c_l_0 = p[22];
    c_l_v = p[23];
    c_l_p = p[24];
    c_l_r = p[25];
    c_l_delta_a = p[26];
    c_l_delta_r = p[27];

    c_n_0 = p[28];
    c_n_v = p[29];
    c_n_p = p[30];
    c_n_r = p[31];
    c_n_delta_a = p[32];
    c_n_delta_r = p[33];

    // Extract state
    double phi, ang_p, ang_r, vel_v;
    vel_v = x[0];
    ang_p = x[1];
    ang_r = x[2];
    phi = x[3];

    // Extract inputs
    // This model assumes the following inputs sequence
    // u = [t delta_a delta_r vel_u vel_w ang_q theta], (Nx7)
    double delta_a, delta_r;
    delta_a = u[0];
    delta_r = u[1];

    // Extract longitudinal states which are taken as inputs
    double vel_u, vel_w, theta, ang_q;
    vel_u = u[2];
    vel_w = u[3];
    ang_q = u[4];
    theta = u[5];

    // Calculate forces and moments
    double V = sqrt(pow(vel_u, 2) + pow(vel_v, 2) + pow(vel_w, 2));
    double dyn_pressure = 0.5 * rho * pow(V, 2);

    // Compute normalized states
    double u_hat = vel_u / V_nom;
    double v_hat = vel_v / V_nom;
    double w_hat = vel_w / V_nom;
    double p_hat = ang_p * (wingspan_m / (2 * V_nom));
    double q_hat = ang_q * (mean_aerodynamic_chord_m / (2 * V_nom));
    double r_hat = ang_r * (wingspan_m / (2 * V_nom));

    double c_Y = c_Y_0 + c_Y_v * v_hat + c_Y_p * p_hat + c_Y_r * r_hat + c_Y_delta_a * delta_a + c_Y_delta_r * delta_r;
    double c_l = c_l_0 + c_l_v * v_hat + c_l_p * p_hat + c_l_r * r_hat + c_l_delta_a * delta_a + c_l_delta_r * delta_r;
    double c_n = c_n_0 + c_n_v * v_hat + c_n_p * p_hat + c_n_r * r_hat + c_n_delta_a * delta_a + c_n_delta_r * delta_r;

    double Y = c_Y * dyn_pressure * planform_sqm;
    double l = c_l * dyn_pressure * planform_sqm * wingspan_m;
    double n = c_n * dyn_pressure * planform_sqm * wingspan_m;

    // Dynamics
    double phi_dot = ang_p + (ang_q * sin(phi) + ang_r * cos(phi)) * tan(theta);
    double p_dot = gam_1 * ang_p * ang_q - gam_2 * ang_q * ang_r + gam_3 * l + gam_4 * n;
    double r_dot = gam_7 * ang_p * ang_q - gam_1 * ang_q * ang_r + gam_4 * l + gam_8 * n;
    double v_dot = ang_p * vel_w - ang_r * vel_u + (1 / mass_kg) * (Y + mass_kg * g * cos(theta) * sin(phi));

    // NOTE: Actuator dynamics are not included here for computational efficiency. The actuators is an isolated system, and is therefore simulated a-priori.

//    dx[0] = c_Y;
//    dx[1] = c_l;
//    dx[2] = c_n;
//    dx[3] = 0;

    dx[0] = v_dot;
    dx[1] = p_dot;
    dx[2] = r_dot;
    dx[3] = phi_dot;
}


void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    /* Declaration of input and output arguments. */
    double t, *x, *u, *params, *dx, *y;
    int     i, np, nu_rows, nu_cols, nx;
    const mxArray *auxvar = NULL; /* Cell array of additional data. */

    if (nrhs < 3) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nrhs",
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
    nu_rows = mxGetM(prhs[2]); /* Number of inputs. */
    nu_cols = mxGetN(prhs[2]); /* Number of inputs. */

    /* Obtain double data pointers from mxArrays. */
    t = mxGetScalar(prhs[0]);  /* Current time value (scalar). */
    x = mxGetPr(prhs[1]);  /* States at time t. */
    u = mxGetPr(prhs[2]);  /* All inputs at all times */
    params = mxGetPr(prhs[3]);  /* Parameters */

    /* Interpolate to find input at the current time */
    double u_at_t[nu_cols - 1]; // t is at the zeroth index in nu_cols
    compute_u_at_t(t, u, u_at_t, nu_rows, nu_cols);

    /* Create matrix for the return arguments. */
    plhs[0] = mxCreateDoubleMatrix(nx, 1, mxREAL);
    dx      = mxGetPr(plhs[0]); /* State derivative values. */

    /* Call function for state derivative update. */
    compute_dx(dx, t, x, u_at_t, params, nu_rows, nu_cols);
}
