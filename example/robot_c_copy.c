/*   Copyright 2005-2015 The MathWorks, Inc. */
/*   Written by Peter Lindskog. */

/* Include libraries. */
#include "mex.h"
#include <math.h>

/* Specify the number of outputs here. */
#define NY 3

/* State equations. */
void compute_dx(double *dx, double *x, double *u, double **p)
{
    /* Declaration of model parameters and intermediate variables. */
    double *g, *Fc, *r, *Im, *m, *pl, *L, *com, *Ia1, *Ia;
    double M[3][3];      /* Mass matrix.                                */
    double Minv[3][3];   /* Inverse of mass matrix.                     */
    double Det;          /* Determinant of M.                           */
    double F[3];         /* Input forces.                               */
    double G[3];         /* Gravitational forces.                       */
    double Gamma[5];     /* Intermediate so-called Christoffel symbols. */
    double H[3];         /* Coriolis and centrifugal forces.            */

    /* Retrieve model parameters. */
    /* The matrices com and Ia are stored columnwise, i.e.,     */
    /* com(1,1) -> com[0], com(2,1) -> com[1],                  */
    /* com(1,2) -> com[3], com(2,2) -> com[3], and so on.       */
    g   = p[0];   /* Gravity constant.                          */
    Fc  = p[1];   /* Voltage-force constant of motor.           */
    r   = p[2];   /* Gear ratio of motor.                       */
    Im  = p[3];   /* Moment of inertia of motor.                */
    m   = p[4];   /* Mass of arm 2 and 3 (incl. tool).          */
    pl  = p[5];   /* Point mass of payload.                     */
    L   = p[6];   /* Length of arm 2 and 3 (incl. tool).        */
    com = p[7];   /* Center of mass coordinates of arm 2 and 3. */
    Ia1 = p[8];   /* Moment of inertia arm 1, element (3,3).    */
    Ia  = p[9];   /* Moment of inertia arm 2 and 3.             */
    
    /* A. Components of the symmetric and positive definite mass matrix M(x, p), a 3x3 matrix. */
    M[0][0] = Ia1[0] + r[0]*r[0]*Im[0] + com[2]*com[2]*m[1] + com[0]*com[0]*m[0] + Ia[2]*pow(cos(x[1]), 2)
              + (Ia[1] + com[1]*com[1]*m[0])*pow(sin(x[1]), 2) + Ia[6]*pow(cos(x[1]+x[2]), 2)
              + Ia[5]*pow(sin(x[1]+x[2]), 2) + m[1]*pow(L[0]*sin(x[1]) + com[3]*sin(x[1]+x[2]), 2)
              + pl[0]*pow(L[0]*sin(x[1]) + L[1]*sin(x[1]+x[2]), 2);
    M[0][2] = (Ia[7] - com[2]*com[3]*m[1])*cos(x[1]+x[2]);
    M[0][1] = (Ia[3] - com[2]*L[0]*m[1] - com[0]*com[1]*m[0])*cos(x[1]) + M[0][2];
    M[1][1] = Ia[4] + Ia[0] + r[1]*r[1]*Im[1] + com[1]*com[1]*m[0] + (com[3]*com[3] + L[0]*L[0])*m[1]
              + (L[1]*L[1] + L[0]*L[0])*pl[0] + 2*(com[3]*L[0]*m[1] + L[1]*L[0]*pl[0])*cos(x[2]);
    M[1][2] = Ia[4] + r[2]*Im[2] + com[3]*com[3]*m[1] + L[1]*L[1]*pl[0]
              + (com[3]*L[0]*m[1] + L[1]*L[0]*pl[0])*cos(x[2]);
    M[2][2] = Ia[4] + r[2]*r[2]*Im[2] + com[3]*com[3]*m[1] + L[1]*L[1]*pl[0];
    
    /* B. Inputs. */
    F[0] = Fc[0]*u[0];
    F[1] = Fc[1]*u[1];
    F[2] = Fc[2]*u[2];
    
    /* C. Gravitational forces G. */
    G[0] = 0;
    G[2] = g[0]*(com[3]*m[1] + L[1]*pl[0])*sin(x[1]+x[2]);
    G[1] = g[0]*(com[1]*m[0] + L[0]*(m[1] + pl[0]))*sin(x[1]) + G[2];
    
    /* D. Coriolis and centrifugal force components Gamma and forces H. */
    Gamma[1] = (Ia[6] - Ia[5] - com[3]*com[3]*m[1] - L[1]*L[1]*pl[0])*sin(x[1]+x[2])*cos(x[1]+x[2])
               - L[0]*(com[3]*m[1] + L[1]*pl[0])*sin(x[1])*cos(x[1]+x[2]);
    Gamma[0] = (Ia[2] - Ia[1] - com[1]*com[1]*m[0] - L[0]*L[0]*(m[1]+pl[0]))*cos(x[1])*sin(x[1])
               - L[0]*(com[3]*m[1] + L[1]*pl[0])*sin(x[1])*cos(x[1]+x[2]) + Gamma[1];
    Gamma[3] = (Ia[7] - com[2]*com[3]*m[1])*sin(x[1]+x[2]);
    Gamma[2] = (Ia[3] - com[2]*L[0]*m[1] - com[0]*com[1]*m[0])*sin(x[1]) + Gamma[3];
    Gamma[4] = L[0]*(com[3]*m[1] + L[1]*pl[0]);
    H[0] = 2*x[0]*(Gamma[0]*x[1] + Gamma[1]*x[2]) + Gamma[2]*x[1]*x[1] + Gamma[3]*(2*x[1] + x[2])*x[2];
    H[1] = -Gamma[0]*x[0]*x[0] + Gamma[4]*(2*x[1] + x[2])*x[2];
    H[2] = -Gamma[1]*x[0]*x[0] - Gamma[4]*x[1]*x[1];
    
    /* E. Compute inverse of M. */
    Det = M[0][0]*M[1][1]*M[2][2] + 2*M[0][1]*M[1][2]*M[0][2]
          - M[0][2]*M[1][1]*M[0][2] - M[0][0]*M[1][2]*M[1][2] - M[0][1]*M[0][1]*M[2][2];
    Minv[0][0] = (M[1][1]*M[2][2] - M[1][2]*M[1][2])/Det;
    Minv[0][1] = (M[0][2]*M[1][2] - M[0][1]*M[2][2])/Det;
    Minv[0][2] = (M[0][1]*M[1][2] - M[0][2]*M[1][1])/Det;
    Minv[1][1] = (M[0][0]*M[2][2] - M[0][2]*M[0][2])/Det;
    Minv[1][2] = (M[0][2]*M[0][1] - M[0][0]*M[1][2])/Det;
    Minv[2][2] = (M[0][0]*M[1][1] - M[0][1]*M[0][1])/Det;
    
    /* State equations. */
    /* x[0]: Relative angle between fundament and arm 1. */
    /* x[1]: Relative angle between arm 1 and arm 2. */
    /* x[2]: Relative angle between arm 2 and arm 3. */
    /* x[3]: Relative velocity between fundament and arm 1. */
    /* x[4]: Relative velocity between arm 1 and arm 2. */
    /* x[5]: Relative velocity between arm 2 and arm 3. */
    dx[0] = x[3];
    dx[1] = x[4];
    dx[2] = x[5];
    dx[3] = Minv[0][0]*(F[0]+G[0]+H[0]) + Minv[0][1]*(F[1]+G[1]+H[1]) + Minv[0][2]*(F[2]+G[2]+H[2]);
    dx[4] = Minv[0][1]*(F[0]+G[0]+H[0]) + Minv[1][1]*(F[1]+G[1]+H[1]) + Minv[1][2]*(F[2]+G[2]+H[2]);
    dx[5] = Minv[0][2]*(F[0]+G[0]+H[0]) + Minv[1][2]*(F[1]+G[1]+H[1]) + Minv[2][2]*(F[2]+G[2]+H[2]);
}

/* Output equations. */
void compute_y(double y[], double x[])
{
    /* y[0]: Relative angle between fundament and arm 1. */
    /* y[1]: Relative angle between arm 1 and arm 2. */
    /* y[2]: Relative angle between arm 2 and arm 3. */
    y[0] = x[0];
    y[1] = x[1];
    y[2] = x[2];
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
    int     i, np;
    size_t  nu, nx;
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
    compute_dx(dx, x, u, p);
    
    /* Call function for output update. */
    compute_y(y, x);
    
    /* Clean up. */
    mxFree(p);
}
