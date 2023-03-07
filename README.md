# Inverted-Pendulum-
Designed a control system for Inverted Pendulum. Also, State Space feedback is designed for the same. 

Inverted Pendulum
This is the proposed model for the system. 

![image](https://user-images.githubusercontent.com/85870494/223322686-6b513cf0-8fa3-4c6b-beb5-f19dcaebedb2.png)






System Parameters:

 ![image](https://user-images.githubusercontent.com/85870494/223322795-237b986b-8905-4c4c-a892-d2ea29a0d2b6.png)

Design Criteria:
 ![image](https://user-images.githubusercontent.com/85870494/223322812-8d5e53f7-ca42-4979-bc05-6667969d20a9.png)

System Equations:
 ![image](https://user-images.githubusercontent.com/85870494/223322851-b0ded727-1fcc-46bc-ad6d-9fd83fb034bd.png)

 

 
Derivation of a Transfer Function:
 
![image](https://user-images.githubusercontent.com/85870494/223322879-d3c59877-7aa8-426d-972c-4176f174fa58.png)
![image](https://user-images.githubusercontent.com/85870494/223322900-0e0d81c0-01d8-438c-b721-6bd1187ebffe.png)
![image](https://user-images.githubusercontent.com/85870494/223322914-728f9769-b8bf-47e0-b014-a3d46da31fc4.png)


Transfer Function:
![image](https://user-images.githubusercontent.com/85870494/223322956-7c7e9674-065c-4851-88b6-10130b49ed10.png)

Implementation of the system on MATLAB:
1.	As discussed above the equation of the system and the transfer function for both the angle of the pendulum and the position of the cart are designed.  
MATLAB code:
%Name: Gayatri Shinde
% Roll No.:42
% PRN: 12010267

M = 0.5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;
q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');

P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);

P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

sys_tf = [P_cart ; P_pend];

inputs = {'u'};
outputs = {'x'; 'phi'};

set(sys_tf,'InputName',inputs)
set(sys_tf,'OutputName',outputs)

sys_tf

rlocus(P_pend)




Output:

![image](https://user-images.githubusercontent.com/85870494/223323013-09b2f2ed-90eb-4bec-b62a-01a739971f2b.png)

 

2.	Root Locus of the system:
 ![image](https://user-images.githubusercontent.com/85870494/223323047-0863e50c-963e-49a8-b2db-0d73eb63b9a3.png)



3.	Open Loop Impulse Response: To verify whether the system is stable or not, the open loop transfer function of the system is obtained.
Code:
%Name: Gayatri Shinde
% Roll No.:42
% PRN: 12010267

t=0:0.01:1;
impulse(sys_tf,t);
title('Open-Loop Impulse Response')
![image](https://user-images.githubusercontent.com/85870494/223323071-ad01da5a-d895-47ea-8afd-4187f54db2f7.png)

  

It can be clearly seen from the ablove plots that the system is unstable as the area is unbounded
 ![image](https://user-images.githubusercontent.com/85870494/223323098-56e33c17-b3db-4887-b176-6b241cfd54a1.png)



4.	Open loop step response: The system is unstable as the area is unbounded. Also, the step info shows the time domain specifications which clearly confirms that the system is unstable. This is the system information for the step response of the plot.

%Name: Gayatri Shinde
% Roll No.:42
% PRN: 12010267
t = 0:0.05:10;
u = ones(size(t));
[y,t] = lsim(sys_tf,u,t);
plot(t,y)
title('Open-Loop Step Response')
axis([0 3 0 50])
legend('x','phi')


 ![image](https://user-images.githubusercontent.com/85870494/223323131-53711e9f-a6c7-418d-8b8e-f90850d65ab4.png)


5.	 To make the system stable and to satisfy all the design criteria, PID controller can be added to the system. 

Kp = 1;
Ki = 1;
Kd = 1;
C = pid(Kp,Ki,Kd);
T = feedback(P_pend,C);
t=0:0.01:10;
impulse(T,t)
title({'Response of Pendulum Position to an Impulse Disturbance';'under PID Control: Kp = 1, Ki = 1, Kd = 1'});

6.	PID controller

The PID controller is designed by giving Kp Ki and Kd values as 1 initially. And the impulse response is generated to verify if the system is stable. 
 ![image](https://user-images.githubusercontent.com/85870494/223323162-c1865076-3104-4d0d-98ed-0b3f024df77a.png)

7.	The transfer function for the same is given as Cascade system transfer function. 

 
![image](https://user-images.githubusercontent.com/85870494/223323178-0e9e3d7b-9511-4417-86e1-5fd759d7a155.png)

8.	It can be seen that the system is still not stable under PID control. So, to make it stable one has modified the values of the gains. For now, the gain Kp is modified and provided to the system and the response is obtained. 

Kp = 100;
Ki = 1;
Kd = 1;
C = pid(Kp,Ki,Kd);
T = feedback(P_pend,C);
t=0:0.01:10;
impulse(T,t)
axis([0, 2.5, -0.2, 0.2]);
title({'Response of Pendulum Position to an Impulse Disturbance';'under PID Control: Kp = 100, Ki = 1, Kd = 1'});

![image](https://user-images.githubusercontent.com/85870494/223323210-295140e9-7cd3-4d92-b200-2b6b08ef86c3.png)

 

9.	From the above response it can be seen that the overshoot criteria for the system have got satisfied. Also, the settling time criteria has also got satisfied. The error is also significantly equal to 0. Therefore, no further integral action is needed. Further the peak response for the system is not satisfied so one has to add the Kd gain with Kp gain. 
 Kp = 100;
Ki = 1;
Kd = 20;
C = pid(Kp,Ki,Kd);
T = feedback(P_pend,C);
t=0:0.01:10;
impulse(T,t)
axis([0, 2.5, -0.2, 0.2]);
title({'Response of Pendulum Position to an Impulse Disturbance';'under PID Control: Kp = 100, Ki = 1, Kd = 20'});


After adding it, the impulse response for the system is found to be. 
 
Now, in this system all the design criteria have been satisfied. No further action is needed. 
![image](https://user-images.githubusercontent.com/85870494/223323235-df8647f7-17be-435c-939a-cfb2f371b1d0.png)

10.	About the cart, the impulse response on the cart’s position can also be found out. 
 ![image](https://user-images.githubusercontent.com/85870494/223323263-4203f013-5cb5-4ca0-9239-b601575d5da8.png)

So, even if the PID stabilizes the response of the angle of the pendulum it doesn’t stabilizes the cart position response. 

11.	Root Locus:
The root locus is another criterion that can be applied to the system and the design criteria can be satisfied. Here the integral action is given to the system and the response is plotted. 
 ![image](https://user-images.githubusercontent.com/85870494/223323277-dd2145d3-76e8-4d2c-af57-eb0886bb2689.png)

 In general, we can pull the branches of our root locus to the left in the complex plane by adding zeros to our system. Adding a zero to our controller will reduce the number of asymptotes from three to two. 

12.	Root locus with PID control:
Zeros are added to the system to make it stable at -3 and -4. 
zeros = zero(C*P_pend)
poles = pole(C*P_pend)
z = [-3 -4];
p = 0;
k = 1;
C = zpk(z,p,k);
rlocus(C*P_pend)
title('Root Locus with PID Controller')
[k,poles] = rlocfind(C*P_pend)

 
A point is selected from the graph above and the step response for the same can be obtained.
![image](https://user-images.githubusercontent.com/85870494/223323290-6ca90db2-aca0-408d-883f-e945d8ed0980.png)
Response: 
 ![image](https://user-images.githubusercontent.com/85870494/223323311-cf91ca1b-eda7-4b83-9266-8f6779f7de77.png)


State Feedback Control
Name: Gayatri Shinde 
Roll no.: 42
PRN: 12010267
Batch: B2 

Advance Control- State Feedback System for inverted Pendulum
Implementation of State Space Method to get all the design criteria of the system fulfilled. 
![image](https://user-images.githubusercontent.com/85870494/223353088-dbb07ddb-c6ec-4599-be11-e03d2b9d5382.png)

1.	Finding open loop poles
For the system, the open loop poles’ locations are to be find. 


poles =

         0
   -0.1428
   -5.6041
    5.5651

One pole with location 5.5651 is in the right side of the s-plane. So, the system is unstable. That is why LQR based controller must be designed for the system. 
         Code:
%Name: Gayatri Shinde
% Roll no. 42
% PRN : 12010267 

M = 0.5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;

p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

poles = eig(A)


2.	Checking the Controllability
Before moving towards the LQR method, first we need to verify whether the given system is controllable or not. It simply means that we have to check if the response of the system can be moved along the graph or not. 

controllability =

     4

So, as the controllability is 4 so we can move to the next part.

Code:
%Name: Gayatri Shinde
% Roll no. 42
% PRN : 12010267 
co = ctrb(sys_ss);
controllability = rank(co)


3.	Now, we have to move for the LQR method. For this, lqr command is there in MATLAB which returns the value of matrix K. Matrix K is basically the lqr of  A B Q and R where Q and R are the error and control effect of the system. 
![image](https://user-images.githubusercontent.com/85870494/223353161-3cf0c268-379f-4bc1-866f-4230060a3d0e.png)

 

The matrix Q is actually  C’* C. 
Code:
%Name: Gayatri Shinde
% Roll no. 42
% PRN : 12010267  
Q = C'*C

![image](https://user-images.githubusercontent.com/85870494/223353202-fd93cef7-0543-4caf-a48f-cb1687fc03d9.png)

 

The K matrix can be found out with this lqr method. Also, the step response with this method can be found out. 

Code:
%Name: Gayatri Shinde
% Roll no. 42
% PRN : 12010267  
Q = C'*C;
R = 1;
K = lqr(A,B,Q,R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r =0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')

 ![image](https://user-images.githubusercontent.com/85870494/223353260-a46f8e02-508d-48a6-b834-b16118da1443.png)


As we can see that the red curve represents the step response of Pendulum’s angle with LQR control and the blue curve represents the step response of Cart position with LQR control. Both the curves show that the overshoot criteria is satisfied but their settling time needs improvement and rise time for the cart’s position need to be improved.  For this the Q (1,1) and Q(3,3) values can be made to 5000 and 100 to get the satisfactory response. 

Code:
%Name: Gayatri Shinde
% Roll no. 42
% PRN : 12010267  
Q = C'*C;
Q(1,1) = 5000;
Q(3,3) = 100
R = 1;
K = lqr(A,B,Q,R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r =0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')
![image](https://user-images.githubusercontent.com/85870494/223353313-3bc4248a-8b97-404e-804d-c57d1a7ac0fe.png)


 


These are the values that are obtained after increasing the values of Q and R. 
 ![image](https://user-images.githubusercontent.com/85870494/223353370-f248eb6a-cd16-43ad-826c-58eeb5820b32.png)

The step response for the same shows that, all the design criteria with resect to the transient time is satisfied.


4.	Adding Precompensation:
Till now, all the transient time criterias are satisfied but the steady state error for the system needs to get reduced. With a full state back controller we can feedback all the states of the system. A new gain N is added to the system after the reference. The value of N can be found out by applying rscale function. 

 
![image](https://user-images.githubusercontent.com/85870494/223353416-966f1137-c982-40fe-8c02-6e52e7f57086.png)

Code:
%Name: Gayatri Shinde
% Roll no. 42
% PRN : 12010267  
Cn = [1 0 0 0];
sys_ss = ss(A,B,Cn,0);
Nbar = rscale(sys_ss,K)

 
Code:
%Name: Gayatri Shinde
% Roll no. 42
% PRN : 12010267  
sys_cl = ss(Ac,Bc*Nbar,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r =0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with Precompensation and LQR Control')
The step response of the system looks like below. 
![image](https://user-images.githubusercontent.com/85870494/223353481-c37be18c-69d4-40e2-891e-306b23acf2e6.png)


 

The steady-state error is within the limits, the rise and settle times are met, and the pendulum's overshoot is within range of the design criteria.

5.	Observer-based control
The state feedback controller is satisfying all the design criteria but was based on the assumptions of full-state feedback. One can also obtain the observer-based control response for the same. 

%Name: Gayatri Shinde
% Roll no. 42
% PRN : 12010267  
Ace = [(A-B*K) (B*K);
       zeros(size(A)) (A-L*C)];
Bce = [B*Nbar;
       zeros(size(B))];
Cce = [Cc zeros(size(Cc))];
Dce = [0;0];

states = {'x' 'x_dot' 'phi' 'phi_dot' 'e1' 'e2' 'e3' 'e4'};
inputs = {'r'};
outputs = {'x'; 'phi'};

sys_est_cl = ss(Ace,Bce,Cce,Dce,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r = 0.2*ones(size(t));
[y,t,x]=lsim(sys_est_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with Observer-Based State-Feedback Control')
![image](https://user-images.githubusercontent.com/85870494/223353591-172fda8c-3b7c-43fa-a307-3e8d5a571af4.png)



