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
    double rho, mass_kg, g, wingspan_m, mean_aerodynamic_chord_m, planform_sqm, V_nom, alpha_nom, delta_e_nom,
        gam_1, gam_2, gam_3, gam_4, gam_5, gam_6, gam_7, gam_8, J_yy, prop_diam_pusher, c_T_pusher;

    rho = p[0];
    mass_kg = p[1];
    g = p[2];
    wingspan_m = p[3];
    mean_aerodynamic_chord_m = p[4];
    planform_sqm = p[5];
    V_nom = p[6];
    alpha_nom = p[7];
    delta_e_nom = p[8];
    gam_1 = p[9];
    gam_2 = p[10];
    gam_3 = p[11];
    gam_4 = p[12];
    gam_5 = p[13];
    gam_6 = p[14];
    gam_7 = p[15];
    gam_8 = p[16];
    J_yy = p[17];
    prop_diam_pusher = p[18];
    c_T_pusher = p[19];

    double c_D_0, c_D_alpha, c_D_alpha_sq, c_D_q_hat, c_D_delta_e, c_D_delta_e_alpha,
        c_L_0, c_L_alpha, c_L_alpha_sq, c_L_q_hat, c_L_delta_e, c_L_delta_e_alpha,
        c_m_0, c_m_alpha, c_m_alpha_sq, c_m_q_hat, c_m_delta_e, c_m_delta_e_alpha;

    c_D_0 = p[20];
    c_D_alpha = p[21];
    c_D_alpha_sq = p[22];
    c_D_q_hat = p[23];
    c_D_delta_e = p[24];
    c_D_delta_e_alpha = p[25];
    c_L_0 = p[26];
    c_L_alpha = p[27];
    c_L_alpha_sq = p[28];
    c_L_q_hat = p[29];
    c_L_delta_e = p[30];
    c_L_delta_e_alpha = p[31];
    c_m_0 = p[32];
    c_m_alpha = p[33];
    c_m_alpha_sq = p[34];
    c_m_q_hat = p[35];
    c_m_delta_e = p[36];
    c_m_delta_e_alpha = p[37];

    // Extract state
    double vel_u, vel_w, ang_q, theta;
    vel_u = x[0];
    vel_w = x[1];
    ang_q = x[2];
    theta = x[3];

    // Extract inputs
    // This model assumes the following inputs sequence
    // u = [t delta_e delta_t vel_v ang_p ang_r phi], (Nx7)
    double delta_e, delta_t;
    delta_e = u[0];
    delta_t = u[1];

    // Extract longitudinal states which are taken as inputs
    double vel_v, ang_p, ang_r, phi;
    vel_v = u[2];
    ang_p = u[3];
    ang_r = u[4];
    phi = u[5];

    // Calculate forces and moments
    double alpha = atan2(vel_w, vel_u) - alpha_nom;
    double V = sqrt(pow(vel_u, 2) + pow(vel_v, 2) + pow(vel_w, 2));
    double dyn_pressure = 0.5 * rho * pow(V, 2);

    // Compute normalized states
    double q_hat = ang_q * (mean_aerodynamic_chord_m / (2 * V_nom));

    double c_D = c_D_0 + c_D_alpha * alpha + c_D_alpha_sq * pow(alpha,2) + c_D_q_hat * q_hat
        + c_D_delta_e * delta_e + c_D_delta_e_alpha * delta_e * alpha;
    double c_L = c_L_0 + c_L_alpha * alpha + c_L_alpha_sq * pow(alpha,2) + c_L_q_hat * q_hat
        + c_L_delta_e * delta_e + c_L_delta_e_alpha * delta_e * alpha;
    double c_m = c_m_0 + c_m_alpha * alpha + c_m_alpha_sq * pow(alpha,2) + c_m_q_hat * q_hat
        + c_m_delta_e * delta_e + c_m_delta_e_alpha * delta_e * alpha;

    double D = c_D * dyn_pressure * planform_sqm;
    double L = c_L * dyn_pressure * planform_sqm;
    double m = c_m * dyn_pressure * planform_sqm * mean_aerodynamic_chord_m;

    double X = -cos(alpha) * D + sin(alpha) * L;
    double Z = -sin(alpha) * D - cos(alpha) * L;
    double T = rho * pow(prop_diam_pusher, 4) * c_T_pusher * delta_t;

    // Dynamics
    double theta_dot = ang_q * cos(phi) - ang_r * sin(phi);
	double q_dot = gam_5 * ang_p * ang_r - gam_6 * (pow(ang_p,2) - pow(ang_r,2)) + (1/J_yy) * m;
    double u_dot = ang_r * vel_v - ang_q * vel_w + (1 / mass_kg) * (X + T - mass_kg * g * sin(theta));
    double w_dot = ang_q * vel_u - ang_p * vel_v + (1 / mass_kg) * (Z + mass_kg * g * cos(theta) * cos(phi));

    // NOTE: Actuator dynamics are not included here for computational efficiency. The actuators is an isolated system, and is therefore simulated a-priori.
    dx[0] = u_dot;
    dx[1] = w_dot;
    dx[2] = q_dot;
    dx[3] = theta_dot;
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
