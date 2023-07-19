% Simscape(TM) Multibody(TM) version: 7.4

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(11).translation = [0.0 0.0 0.0];
smiData.RigidTransform(11).angle = 0.0;
smiData.RigidTransform(11).axis = [0.0 0.0 0.0];
smiData.RigidTransform(11).ID = '';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [-237.30594195213928 140.96818335894801 -15.422794350706926];  % mm
smiData.RigidTransform(1).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(1).axis = [0.57735026918962584 -0.57735026918962595 0.57735026918962562];
smiData.RigidTransform(1).ID = 'B[Y-Axis-1:-:Z-Axis-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [-237.30594195213951 143.81823389725156 -15.422794350706909];  % mm
smiData.RigidTransform(2).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(2).axis = [0.57735026918962584 -0.57735026918962595 0.57735026918962562];
smiData.RigidTransform(2).ID = 'F[Y-Axis-1:-:Z-Axis-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [-487.29557504353318 49.268183358946224 53.577205649298243];  % mm
smiData.RigidTransform(3).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(3).axis = [0.57735026918962595 0.57735026918962562 0.57735026918962584];
smiData.RigidTransform(3).ID = 'B[X-Axis-1:-:Y-Axis-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [-492.3059419521328 49.268183358943759 53.577205649298207];  % mm
smiData.RigidTransform(4).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(4).axis = [0.57735026918962595 0.57735026918962562 0.57735026918962584];
smiData.RigidTransform(4).ID = 'F[X-Axis-1:-:Y-Axis-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [-84.79557504353042 -48.731816641052802 442.19209640970746];  % mm
smiData.RigidTransform(5).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(5).axis = [1 0 0];
smiData.RigidTransform(5).ID = 'B[Fixed-1:-:X-Axis-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [-84.795575043535791 -48.731816641056867 449.57720564940337];  % mm
smiData.RigidTransform(6).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(6).axis = [-1 0 0];
smiData.RigidTransform(6).ID = 'F[Fixed-1:-:X-Axis-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [-268.29557504353392 -39.881816641053447 -273.807903590313];  % mm
smiData.RigidTransform(7).angle = 3.1415926535897927;  % rad
smiData.RigidTransform(7).axis = [-1 -0 -0];
smiData.RigidTransform(7).ID = 'B[Fixed-1:-:Feeder-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [-268.29557504353392 -39.881816641053376 -280.85806506722236];  % mm
smiData.RigidTransform(8).angle = 3.1415926535897927;  % rad
smiData.RigidTransform(8).axis = [-1 0 0];
smiData.RigidTransform(8).ID = 'F[Fixed-1:-:Feeder-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [593.74338973685678 477.33223167180898 741.97189282525392];  % mm
smiData.RigidTransform(9).angle = 0;  % rad
smiData.RigidTransform(9).axis = [0 0 0];
smiData.RigidTransform(9).ID = 'RootGround[Fixed-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(10).translation = [325.44781469331804 410.4504150307564 603.1639892349408];  % mm
smiData.RigidTransform(10).angle = 2.2286131332297372;  % rad
smiData.RigidTransform(10).axis = [-0.49108444480938246 -0.61596918269758472 -0.61596918269758461];
smiData.RigidTransform(10).ID = 'SixDofRigidTransform[product_2-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(11).translation = [325.44781469331787 500.45041503075652 603.16398923494035];  % mm
smiData.RigidTransform(11).angle = 1.7140643245482803;  % rad
smiData.RigidTransform(11).axis = [0.86609503094148543 0.3534681013744152 -0.3534681013744147];
smiData.RigidTransform(11).ID = 'SixDofRigidTransform[product_1-1]';


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(7).mass = 0.0;
smiData.Solid(7).CoM = [0.0 0.0 0.0];
smiData.Solid(7).MoI = [0.0 0.0 0.0];
smiData.Solid(7).PoI = [0.0 0.0 0.0];
smiData.Solid(7).color = [0.0 0.0 0.0];
smiData.Solid(7).opacity = 0.0;
smiData.Solid(7).ID = '';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0;  % kg
smiData.Solid(1).CoM = [0 0 0];  % mm
smiData.Solid(1).MoI = [0 0 0];  % kg*mm^2
smiData.Solid(1).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(1).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = 'Feeder*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.098174770424681035;  % kg
smiData.Solid(2).CoM = [0 0 25.000000000000007];  % mm
smiData.Solid(2).MoI = [35.792885050664971 35.792885050664971 30.679615757712831];  % kg*mm^2
smiData.Solid(2).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(2).color = [0.32941176470588235 0.31764705882352939 1];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = 'product_2*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 0;  % kg
smiData.Solid(3).CoM = [0 0 0];  % mm
smiData.Solid(3).MoI = [0 0 0];  % kg*mm^2
smiData.Solid(3).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(3).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = 'Z-Axis*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 0;  % kg
smiData.Solid(4).CoM = [0 0 0];  % mm
smiData.Solid(4).MoI = [0 0 0];  % kg*mm^2
smiData.Solid(4).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(4).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = 'X-Axis*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(5).mass = 0.078539816339744828;  % kg
smiData.Solid(5).CoM = [0 0 20];  % mm
smiData.Solid(5).MoI = [22.743821815051117 22.743821815051113 24.543692606170264];  % kg*mm^2
smiData.Solid(5).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(5).color = [1 0 0];
smiData.Solid(5).opacity = 1;
smiData.Solid(5).ID = 'product_1*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(6).mass = 0;  % kg
smiData.Solid(6).CoM = [0 0 0];  % mm
smiData.Solid(6).MoI = [0 0 0];  % kg*mm^2
smiData.Solid(6).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(6).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(6).opacity = 1;
smiData.Solid(6).ID = 'Y-Axis*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(7).mass = 0;  % kg
smiData.Solid(7).CoM = [0 0 0];  % mm
smiData.Solid(7).MoI = [0 0 0];  % kg*mm^2
smiData.Solid(7).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(7).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(7).opacity = 1;
smiData.Solid(7).ID = 'Fixed*:*Default';


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the PrismaticJoint structure array by filling in null values.
smiData.PrismaticJoint(4).Pz.Pos = 0.0;
smiData.PrismaticJoint(4).ID = '';

smiData.PrismaticJoint(1).Pz.Pos = 0;  % m
smiData.PrismaticJoint(1).ID = '[Y-Axis-1:-:Z-Axis-1]';

smiData.PrismaticJoint(2).Pz.Pos = 0;  % m
smiData.PrismaticJoint(2).ID = '[X-Axis-1:-:Y-Axis-1]';

smiData.PrismaticJoint(3).Pz.Pos = 0;  % m
smiData.PrismaticJoint(3).ID = '[Fixed-1:-:X-Axis-1]';

smiData.PrismaticJoint(4).Pz.Pos = 0;  % m
smiData.PrismaticJoint(4).ID = '[Fixed-1:-:Feeder-1]';

