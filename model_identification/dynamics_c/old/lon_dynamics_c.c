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

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
#define bound(x,bl,bu) (min(max(x,bl),bu))

double interp(double x0, double x1, double y0, double y1, double xp)
{
		double yp = y0 + ((y1 - y0) / (x1 - x0)) * (xp - x0);
		return yp;
}

void compute_u_at_t(double t, double *u, double *u_at_t, int nu_rows, int nu_cols)
{
		int interp_end_index = 0;
		double curr_t = u[interp_end_index];
		while (true)
		{
			curr_t = u[interp_end_index];
			// Stop looking if we have moved past t or will go to the end
			if ((curr_t > t) || (interp_end_index + 1 >= nu_rows))
				break;
			++interp_end_index;
		}

		for (int col_i = 0; col_i < nu_cols; ++col_i)
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
			servo_time_const_s, servo_rate_lim_rad_s,
			prop_diam_pusher_four, c_T_pusher,
			gam_1, gam_2, gam_3, gam_4, gam_5, gam_6, gam_7, gam_8, J_yy;

		rho = p[0];
		mass_kg = p[1];
		g = p[2];
		wingspan_m = p[3];
		mean_aerodynamic_chord_m = p[4];
		planform_sqm = p[5];
		V_nom = p[6];
		servo_time_const_s = p[7];
		servo_rate_lim_rad_s = p[8];
		prop_diam_pusher_four = p[9];
		c_T_pusher = p[10];
		gam_1 = p[11];
		gam_2 = p[12];
		gam_3 = p[13];
		gam_4 = p[14];
		gam_5 = p[15];
		gam_6 = p[16];
		gam_7 = p[17];
		gam_8 = p[18];
		J_yy = p[19];

		double c_X_0, c_X_q, c_X_u, c_X_w, c_X_w_sq, c_X_delta_e,
					 c_Z_0, c_Z_w, c_Z_delta_e,
					 c_m_0, c_m_q, c_m_w, c_m_delta_e, c_m_delta_e_sq, c_m_delta_r_sq;

		c_X_0 = p[20];
		c_X_q = p[21];
		c_X_u = p[22];
		c_X_w = p[23];
		c_X_w_sq = p[24];
		c_X_delta_e = p[25];

		c_Z_0 = p[26];
		c_Z_w = p[27];
		c_Z_delta_e = p[28];

		c_m_0 = p[29];
		c_m_q = p[30];
		c_m_w = p[31];
		c_m_delta_e = p[32];
		c_m_delta_e_sq = p[33];
		c_m_delta_r_sq = p[34];


		// Extract state
    double theta, ang_q, vel_u, vel_w;
    theta = x[0];
    ang_q = x[1];
    vel_u = x[2];
    vel_w = x[3];

		// Extract inputs
    double delta_a, delta_e, delta_r, n_p;
    delta_a = u[0];
    delta_e = u[1];
    delta_r = u[2];
    n_p = u[3];

		// Extract lateral states which are taken as inputs
		double phi, psi, ang_p, ang_r, vel_v;
		phi = u[4];
		psi = u[5];
		ang_p = u[6];
		ang_r = u[7];
		vel_v = u[8];

		// Calculate forces and moments
    double V = sqrt(pow(vel_u, 2) + pow(vel_v, 2) + pow(vel_w, 2));
    double dyn_pressure = 0.5 * rho * pow(V, 2);

    double u_hat = vel_u / V_nom;
    double v_hat = vel_v / V_nom;
    double w_hat = vel_w / V_nom;
    double p_hat = ang_p * (wingspan_m / (2 * V_nom));
    double q_hat = ang_q * (mean_aerodynamic_chord_m / (2 * V_nom));
    double r_hat = ang_r * (wingspan_m / (2 * V_nom));

    double c_X = c_X_0 + c_X_q * q_hat + c_X_u * u_hat + c_X_w * w_hat + c_X_w_sq * pow(w_hat,2) + c_X_delta_e * delta_e;
    double c_Z = c_Z_0 + c_Z_w * w_hat + c_Z_delta_e * delta_e;
    double c_m = c_m_0 + c_m_q * q_hat + c_m_w * w_hat + c_m_delta_e * delta_e + c_m_delta_e_sq * pow(delta_e,2) + c_m_delta_r_sq * pow(delta_r,2);

    double X = c_X * dyn_pressure * planform_sqm;
    double Z = c_Z * dyn_pressure * planform_sqm;
    double M = c_m * dyn_pressure * planform_sqm * mean_aerodynamic_chord_m;

    double T = rho * prop_diam_pusher_four * c_T_pusher * pow(n_p, 2);

		// Dynamics
    double theta_dot = ang_q * cos(phi) - ang_r * sin(phi);
	  double q_dot = gam_5 * ang_p * ang_r - gam_6 * (pow(ang_p,2) - pow(ang_r,2)) + (1/J_yy) * M;
    double u_dot = ang_r * vel_v - ang_q * vel_w + (1 / mass_kg) * (X + T - mass_kg * g * sin(theta));
    double w_dot = ang_q * vel_u - ang_p * vel_v + (1 / mass_kg) * (Z + mass_kg * g * cos(theta) * cos(phi));

		// NOTE: Actuator dynamics are not included here for computational efficiency. The actuators is an isolated system, and is therefore solved a-priori.

		dx[0] = theta_dot;
		dx[1] = q_dot;
		dx[2] = u_dot;
		dx[3] = w_dot;
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
		double u_at_t[nu_cols];
		compute_u_at_t(t, u, u_at_t, nu_rows, nu_cols);

    /* Create matrix for the return arguments. */
    plhs[0] = mxCreateDoubleMatrix(nx, 1, mxREAL);
    dx      = mxGetPr(plhs[0]); /* State derivative values. */

    /* Call function for state derivative update. */
    compute_dx(dx, t, x, u_at_t, params, nu_rows, nu_cols);
}
