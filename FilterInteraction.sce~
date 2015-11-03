// This file will eventually become a filter interaction simulation.
//  For now, it will test the github.
// To run on linux, type :!scilab-cli -f %
// To run on windows, type !C:\"Program Files"\scilab-5.5.1\bin\scilex  -nwni -f %
// Added a line. 11/2/2015
printf("What hath God wrought.\n");
L=1;
C=1;
// Scaling.  Scaling can be done for frequency, current, and voltage.
//  To scale frequency, L=L*f/fnew
//  Capacitance Cnew=Cold*f/fnew
//  The base frequecy for the L=1, C=1 is 1 rad/sec.
//
R=0.1
//Simulate a simple RCL filter.
//  x0 = Il
//  x1 = vc
//
A = [-R -1/L;1/C 0]
B = [1;0]
C = [0 1]
D = [0]
// The output is the capacitor voltage.
Gfilt = syslin('c',A,B,C,D);
t=(0:%pi/100:20*%pi)
u=ones(t)
y=csim(u,t,Gfilt)
plot2d(t,y)
xgrid()

//Now I want to explore the two methods of controlling the LC filter.
//    1.  Classical control using lead compensation.
//    2.  Modern control using state variable feedback and a PI controller.
//  I want to learn the limitations of control using the two methods.
//  Can you in theory or in practice obtain better stability with one method or
//  the other?  Are they in fact equivalent?  Is lead compensation a subset
//  of state variable feedback?
//
