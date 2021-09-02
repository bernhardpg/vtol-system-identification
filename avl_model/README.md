## Loading the VTOL model into AVL
run avl
`load vtol.avl` loads the model itself
`case vtol.run` loads the run cases, such as trim and zero-lift moment
`mass vtol.mass` loads the inertia properties of the aircraft
`mset 0` applies the inertia properties to the run cases
`oper` to enter the operations menu

## Showing Load distribution
`oper`
`g` to open the graphics window 
`lo` to show the- loading

## Eigenmode analysis
`MODE`
`#` replace # with desired run case
`n` to display pole map 
`x` to enter interactive mode where you can click modes
click the desired pole to investigate it

## State space matrix
`MODE`
`#` replace # with desired run case
`s` Gives the system matrix
