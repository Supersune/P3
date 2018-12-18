%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% INVERSE DYNAMICS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Symbolic variables

syms theta real;
syms Q1 real; %Angular position
syms Q2 real;
syms Q3 real;

syms Q1d real; %Velocity
syms Q2d real;
syms Q3d real;

syms Q1dd real; %Acceleration
syms Q2dd real;
syms Q3dd real;
syms G real; %Gravity
    

%% Rotation matrices

rotz(theta) = [cos(theta) -sin(theta) 0;
    sin(theta) cos(theta) 0;
    0 0 1];

roty(theta) = [cos(theta) 0 sin(theta);
    0 1 0;
    -sin(theta) 0 cos(theta)];

rotx(theta)=[1 0 0;
    0 cos(theta) -sin(theta);
    0 sin(theta) cos(theta)]; %positive

%% Manipulator properties (Inertia tensor, mass matrix, etc.)

    %g = [0; 0; 9.81]; %gravity vector
    g = [0; 0; G]; %Gravity vector
    z = [0; 0; 1]; %z unit vector
    Q_dotdot = [Q1dd; Q2dd; Q3dd]; %Acceleration vecotr
    
    %Masses of links
    M1 = 0.21661; %kg
    M2 = 0.21537;
    M3 = 0.27168;
    
    %Inertia tensors of the 3 links    
    
    J_1 = [408697.54/1000000000 -79590.03/1000000000 151.28/1000000000;
        -79590.03/1000000000 180949.10/1000000000 -642.37/1000000000;
        151.28/1000000000 -642.37/1000000000 468214.2/1000000000];
    
    J_2 = [54456.23/1000000000 -14430.59/1000000000 0;
        -14430.59/1000000000 6279462.52/1000000000 0;
        0 0 6258602.44/1000000000;];
    
    J_3 = [142258.96/1000000000 -6663.23/1000000000 923.05/1000000000;
        -6663.23/1000000000 4161452.68/1000000000 6.81/1000000000;
        923.05/1000000000 6.81/1000000000 4073862.02/1000000000];
    
    %Position vectors of links in zero position (theta = 0)
    L1 = [0; 0; 0.020];
    L2 = [0.21882; 0; 0];
    L3 = [0.242; 0; 0];
    
    %Position vectors of links center of mass in zero position (theta = 0)
    L1_COM = [0; 0; 0.02]; % 2
    L2_COM = [0.16010; 0; 0];
    L3_COM = [0.1162; 0; 0];
    
 
%% Position vectors

    % Notation %
    % s_i = local frame end effector position of link i
    % sc_i = local frame center of mass position of link i
    % Ri = rotation matrix of link i to move it into the coordinate system
    % of link 1
    % r_i = end effector position of link i in the global frame
    % rc_i = center of mass position of link i in the global frame

    %Link 1
    s_1 = L1
    sc_1 = s_1/2
    
    %Link 2
    R2 = rotz(Q1)*rotx(deg2rad(90))*rotz(Q2) 
    s_2 = R2*   L2 %Cartesian configuration of link 2's end effector
    sc_2 = R2*(L2_COM) %Cartesian configuration of link 2's center of mass
    
    %Link 3
    R3 = rotz(Q1)*rotx(deg2rad(90))*rotz(Q2+Q3) 
    s_3 = R3*L3 %Cartesian configuration of link 3's end effector
    sc_3 = R3*(L3_COM) %Cartesian configuration of link 3's center of mass
    
    %Global position vectors
    
    r_1 = vpa(s_1,2) %End-effector position vector
    r_2 = vpa(r_1+s_2,2) %End-effector position vector
    r_3 = vpa(r_2+s_3,2) %End-effector position vector
    
    rc_1 = vpa(sc_1,2) %COM position vector
    rc_2 = vpa(r_1+sc_2,2) %COM position vector
    rc_3 = vpa(r_2+sc_3,2) %COM position vector
    %r=vpa(simplify([rc_1;rc_2;rc_3]),2)

%% Velocities

    %Angular velocities
    omega_1 = [0; 0; Q1d]
    omega_2 = Q2d*[R2(1,3); R2(2,3); R2(3,3)]+omega_1
    omega_3 = Q3d*[R3(1,3); R3(2,3); R3(3,3)]+omega_2

    %Velocities of end-effectors
    v_1 = vpa(cross(omega_1,s_1),2)
    v_2 = vpa(v_1 + cross(omega_2,s_2),2)
    %Not printed, too large, see matlab file instead.
    v_3 = vpa(v_2 + cross(omega_3,s_3),2);
    
    %Velocities of COM
    vc_1 = vpa(cross(omega_1,sc_1),2)
    vc_2 = vpa(v_1 + cross(omega_2,sc_2),2)
    %Not printed, too large, see matlab file instead.
    vc_3 = vpa(v_2 + cross(omega_3,sc_3),2);
   
    
%% Lagrangian dynamics

    % Link 1
    %Not printed, too large, see matlab file instead.
    T1 = vpa(1/2*M1*dot(transpose(vc_1),vc_1)  + (0.5* dot((J_1*omega_1),transpose(omega_1))),2); %Kinetic energy
    V1 = vpa(M1*dot(transpose(g),rc_1),2); %Potential energy 
    
    % Link 2
    %Not printed, too large, see matlab file instead.
    J_2_O = vpa(R2*J_2*transpose(R2),2);
    T2 = vpa((1/2*M2*dot(transpose(vc_2),vc_2))+(0.5*dot((J_2_O*omega_2),transpose(omega_2))),2); %Kinetic energy
    V2 = vpa(M2*dot(transpose(g),rc_2),2); %Potential energy
    
    % Link 3
    %Not printed, too large, see matlab file instead.
    J_3_O = vpa(R3*J_3*transpose(R3),2);
    T3 = vpa((1/2*M3*dot(transpose(vc_3),vc_3))+(0.5*dot((J_3_O*omega_3),transpose(omega_3))),2) %Kinetic energy;
    V3 = vpa(M3*dot(g,transpose(rc_3)),2) %Potential energy;

    %Lagrangian
    %Not printed, too large, see matlab file instead.
    L = vpa(T1 - V1 + T2-V2 + T3-V3,2);
    
    pD_T1 = vpa(diff(L, Q1d),2);
    d_pD_T1 = diff(pD_T1, Q1)*Q1d+diff(pD_T1,Q2) *Q2d + diff(pD_T1,Q3)*Q3d + diff(pD_T1,Q1d)*Q1dd + diff(pD_T1,Q2d)*Q2dd + diff(pD_T1,Q3d)*Q3dd;
    pD_V1 = diff(L, Q1);
    
    pD_T2 = diff(L, Q2d);
    d_pD_T2 = diff(pD_T2, Q1)*Q1d+diff(pD_T2,Q2) *Q2d + diff(pD_T2,Q3)*Q3d + diff(pD_T2,Q1d)*Q1dd + diff(pD_T2,Q2d)*Q2dd + diff(pD_T2,Q3d)*Q3dd;
    pD_V2 = diff(L, Q2);
     
    pD_T3 = diff(L, Q3d);
    d_pD_T3 = diff(pD_T3, Q1)*Q1d+diff(pD_T3,Q2) *Q2d + diff(pD_T3,Q3)*Q3d + diff(pD_T3,Q1d)*Q1dd + diff(pD_T3,Q2d)*Q2dd + diff(pD_T3,Q3d)*Q3dd;
    pD_V3 = diff(L, Q3);
    
    tau_1 = simplify(d_pD_T1-pD_V1);
    tau_2 = simplify(d_pD_T2-pD_V2);
    tau_3 = simplify(d_pD_T3-pD_V3);
    
    % Symbolic torque equation derived using lagrangian approach
    % Not printed, too large, see matlab file instead.
    Tau_syms=vpa([tau_1;tau_2;tau_3],3);
    
    %% Symbolic state space equation
    
    %The entries of the mass matrix
    M(1,1) = vpa(simplify(diff(tau_1, Q1dd)),2);
    M(1,2) = vpa(simplify(diff(tau_1, Q2dd)),2);
    M(1,3) = vpa(simplify(diff(tau_1, Q3dd)),2);
    
    M(2,1) = vpa(simplify(diff(tau_2, Q1dd)),2);
    M(2,2) = vpa(simplify(diff(tau_2, Q2dd)),2);
    M(2,3) = vpa(simplify(diff(tau_2, Q3dd)),2);
    
    M(3,1) = vpa(simplify(diff(tau_3, Q1dd)),2);
    M(3,2) = vpa(simplify(diff(tau_3, Q2dd)),2);
    M(3,3) = vpa(simplify(diff(tau_3, Q3dd)),2);
    
    %Mass matrix
    %Not printed, too large, see matlab file instead.
    MassMatrix = vpa(M,2)
    
    ang1 = 0;
    ang2 = 0;
    ang3 = 0;
    
    vel1 = 0;
    vel2 = 0;
    vel3 = 0;
    
    acc1 = 0;
    acc2 = 0;
    acc3 = 0;
    
    %Finding the viscous friction and coulumb vector
    tv1 = subs(tau_1,[Q1d,Q2d,Q3d],[vel1,vel2,vel3]);
    tv2 = subs(tau_2,[Q1d,Q2d,Q3d],[vel1,vel2,vel3]);
    tv3 = subs(tau_3,[Q1d,Q2d,Q3d],[vel1,vel2,vel3]);
    
    V(1,1) = tau_1 - tv1;
    V(2,1) = tau_2 - tv2;
    V(3,1) = tau_3 - tv3;
    
    %Viscous friction and coulumb vector
    %Not printed, too large, see matlab file instead.
    ViscousFrictionAndCoulumbVector = vpa(V,2)
    
    %Finding the gravity vector
    GravVec(1,1) = vpa(simplify(diff(tau_1,G)),2)*G;
    GravVec(2,1) = vpa(simplify(diff(tau_2,G)),2)*G;
    GravVec(3,1) = vpa(simplify(diff(tau_3,G)),2)*G;
    
    %Gravity Vector
    %Not printed, too large, see matlab file instead.
    GravityVector = vpa(GravVec,2)
    
    %State space equation
    %Not printed, too large, see matlab file instead.
    Tau = M*Q_dotdot + V + GravVec;
    
    %Print state space equation
    %Not printed, too large, see matlab file instead.
    Torque = vpa(Tau,2);

    %% Compute torque values
    
    % Choose values
    
    %angular positions
    ang1 = 0;
    ang2 = -1;
    ang3 = 0.3;
    
    %angular velocities
    vel1 = 2;
    vel2 = 5;
    vel3 = 1;
    
    %angular acceleration
    acc1 = 1;
    acc2 = 2;
    acc3 = 5;
    
    gConst = [0; 0; 9.81];
    gConst = 9.81;
    
    Tau1 = subs(Tau(1,1),[Q1,Q2,Q3,Q1d,Q2d,Q3d,Q1dd,Q2dd,Q3dd,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,gConst]);
    Tau2 = subs(Tau(2,1),[Q1,Q2,Q3,Q1d,Q2d,Q3d,Q1dd,Q2dd,Q3dd,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,gConst]);
    Tau3 = subs(Tau(3,1),[Q1,Q2,Q3,Q1d,Q2d,Q3d,Q1dd,Q2dd,Q3dd,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,gConst]);
    
    ComputedTorque = [Tau1; Tau2; Tau3];
    
    %print computed torque
    vpa(ComputedTorque,3)
    
    Acc = inv(M)*(ComputedTorque-(V+GravVec));
    Acc1 = vpa(subs(Acc(1,1),[Q1,Q2,Q3,Q1d,Q2d,Q3d,G],[ang1,ang2,ang3,vel1,vel2,vel3,gConst]),2)
    Acc2 = vpa(subs(Acc(2,1),[Q1,Q2,Q3,Q1d,Q2d,Q3d,G],[ang1,ang2,ang3,vel1,vel2,vel3,gConst]),2)
    Acc3 = vpa(subs(Acc(3,1),[Q1,Q2,Q3,Q1d,Q2d,Q3d,G],[ang1,ang2,ang3,vel1,vel2,vel3,gConst]),2)

    