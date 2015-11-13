//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//  This file generates the control matrices for the fan control on the Subarc Turbo product.
//----------------------------------------------------------------------------------------
//  The line below (without the beginning slashes)  executes this script from 
//  the vim command line.  
//     !C:\"Program Files"\scilab-5.5.1\bin\scilex  -nwni -f %
//  To run with graphics capability use the command below.
//     !C:\"Program Files"\scilab-5.5.1\bin\scilex  -nw -f %
s         = %s;               //SciLAB overhead
z         = %z;               //SciLAB overhead
Tsamp     = 12.5e-6;          //Sample Period
L         = 33.7e-6;          //first guess at output inductance.
Cf        = 880e-6;           //Capacitor size.
R         = 28.0;             // Estimated load resistance.
w0        = 1/sqrt(L*Cf);

//----------------------------------------------------------------------------------------
//  Compensation for a 2nd order LC filter consists of the following singularities:
//    p1  : integrator pole at the origin.
//    kp  : gain term.
//    z1  : zero to cancel the phase shift of the integrator.
//    z2  : zero to provide lead compensation before the LC resonance frequency.
//    p2  : rolloff to reduce gain above the LC resonance.
//
//----------------------------------------------------------------------------------------

Kp        = 50;
p1        = 24;
z1        = w0/5;
z2        = w0/10;
p2        = w0*8;
p3        = w0*20;
Gc        = Kp* (1+s/z1)*(1+s/z2)/((1+s/p1)*(1+s/p2)*(1+s/p3));
Gc_old    = 1000* (1+s/z1)*(1+s/z2)/((s)*(1+s/p2)*(1+s/p3));
//  The next several lines apply linear transformations to
//  the system in order to make calculations easier later on.
Gss       = tf2ss(Gc);       //Convert to state space
Gss(7)    = 'c'
[A,B,C,D] = abcd(Gss);        //Extract the matrices
[A,P]     = bdiag(Gss(2));    //diagonalize the A matrix   
B         = inv(P)*B;         //P is the linear transformation matrix.
C         = C*P;
//P2        = diag(ones(C)./C); //P2: linear transformation so C is all ones.
//A         = inv(P2)*A*P2;     //Transform A
//B         = inv(P2)*B;        //Transform B for consistency.
//C         = C*P2;
Gs2       = syslin('c',A,B,C,D); //Use the new linear system for further investigation.
f         = logspace(-1,5,1000);

outputfile = mopen("C:\Users\julric\Desktop\temp.txt",'w')
mfprintf(outputfile,".param a11=%f, a22=%f, a33=%f \n", A(1,1),A(2,2),A(3,3))
mfprintf(outputfile,".param b11=%f, b21=%f, b31=%f \n",B(1),B(2),B(3))
mfprintf(outputfile,".param c11=%f, c12=%f, c13=%f d=%f \n",C(1),C(2),C(3),D)

bode(Gss,f);       //These commented lines were used to verify that Gc= Gc2
xset('window',2);
Gs_old=tf2ss(Gc_old)
Gs_old(7)='c'
bode(Gs_old,f);


//bode(Gs2,f)

//----------------------------------------------------------------------------------------
//  Now convert Gc to discrete time state space system Gcz.
//  The control law is: x[k+1] = A*x[k] + B*u;
//  The output eqn is:  y[k]  = C*x[k] + D*u;
//  Where u is the input, y is the output, and x is the state matrix.
//  All coefficients are powers of 2 or 1- a power of 2.
//----------------------------------------------------------------------------------------
Gcz       = cls2dls(Gs2,Tsamp);
u         = ones(1,1000);

//Simulate the continuous time system to compare to spice.
//
t         = (0:1e-4:500e-3)';
u         = ones(t);
y         = csim(u',t',Gs2);
//xset('window',2)

y         = dsimul(Gcz,u');
//plot2d(t',y')
//legend('$x_0$','$x_1$')
//xgrid()
x         = 0.012/Tsamp;
//disp([time(x+1) y(1,x+1) y(2,x+1)])
//----------------------------------------------------------------------------------------
//    Simulate the output filter.
//----------------------------------------------------------------------------------------
Gfilt     = (R/(1+s*Cf*R))/(R/(1+s*Cf*R)+s*L)
//xset('window',3);
//Gfss      = syslin('c',Gfilt);
//bode(Gfss,f);

//----------------------------------------------------------------------------------------
//    Simulate the system with output filter.
//----------------------------------------------------------------------------------------
Gsys      = Gc*Gfilt
//xset('window',4);
Gsss      = syslin('c',Gsys);
//bode(Gsss,f);
//  This section shows the resulting controller.  Trust but verify!
//!lss  A  B  C  D  X0  dt  !
/// 
//       Gcz(2) = A matrix = 
// 
//    1.    0.           0.         
//    0.    0.5398317    0.         
//    0.    0.           0.7864523  
// 
//       Gcz(3) = B matrix = 
// 
//    0.000129   
//    1.0043226  
//  - 0.5285098  
// 
//       Gcz(4) = C matrix = 
// 
//    1.    1.    1.  
// 
//       Gcz(5) = D matrix = 
// 
//    0.3564501  
//
printf("  Tsamp %f\n",Tsamp)
disp(Gcz) 

// Linear system to describe LCR circuit.
// x0= inductor current.  x1 = capacitor voltage.
A=[-1/L 0;1/Cf -1/R/Cf]
B=[1/L;0]
C=[0 1]
D=0;
Gfilt=syslin('c',A,B,C,D)
[A,B,C,D]=abcd(Gcz);

//The following section was used to troubleshoot the simulated VHDL code.
//Set up the matrices and input the state variables from the simulation
D=1/4
A= [1 0 0;0 0.5 0; 0 0 0.75]
B = [ 1/1024;1;-0.5]
X=[-3544414;-83967;83968]
ref = 2600
act = 2641
u=1024*(ref-act)
Xnew = A*X + B*u
Xnew = min(Xnew,ones(Xnew)*2^19)  //Clamp Xnew to upper limit
Xnew(3) = max(Xnew(3),(-100000))
ynew = (C*Xnew+D*u)/1024 + ref*1.25;
disp(Xnew)
printf("U calculated %f \n",u)
printf("The new y value is %f\n",ynew)

close(outputfile)

