## Loading the VTOL model into AVL
run avl
`load vtol.avl` loads the model itself
`case vtol.run` loads the run cases, such as trim and zero-lift moment
`mass vtol.mass` loads the inertia properties of the aircraft
`mset 0` applies the inertia properties to the run cases
`oper` to enter the operations menu
