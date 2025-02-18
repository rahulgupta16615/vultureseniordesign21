%THE VULTURE: Stability & Control Parameters
%SOURCES:
    %[1]NICOALI Txt: Ch.21 (Satbility & Controls), Ch.22 (Trim Drag & Maneuv. Flight) & Ch.23 (Control Surface sizing criteria)
    %[2]RAYMER Txt: Ch.16 (Stability, Control & Handling qualities)
    %[3]GUDMUNDSON Txt: Ch.12 (The Anatomy of the Tail)
    %[4]SADREYS Txt: Ch.6 (Tail Design) & Ch.12 (Design of Control surfaces)
    %[5]KUNDU Txt: Ch.12 (Stability Considerations affecting aircraft)
    %[6]Boeing S&C Notes -(Empenage design)
    %[7]Online ref. for rudder sizing: https://www.ripublication.com/ijaer18/ijaerv13n10_85.pdf
    %[8]Online ref. for aileron sizing: http://www.flightlevelengineering.com/downloads/ProAdvice%203%20-%20AILERON%20SIZING.pdf
    %[9]Online ref. for elevator sizing: http://www.engineeringscience.rs/images/pdf/14829.pdf
    %[10]Online ref. for estmating Xn & SM: https://ciurpita.tripod.com/rc/notes/neutralPt.html
    %[11]Online ref. for estimating flaps: https://www.fzt.haw-hamburg.de/pers/Scholz/HOOU/AircraftDesign_8_HighLift.pdf
    %[12] Online ref. for nv est/empenage stuff: https://nptel.ac.in/content/storage2/courses/101106043/downloads/008_Chapter%202_L6.pdf
    clear;clc;
%------------------------------------------------------------------------------------------%


%------------------------------------------------------------------------------------------%
%%
%Tail sizing parameters
cg2 = 15.1086; %ft, aft from the nose
sw_2 = 295; %Wing planform area from VSP in ft^2
cw_2 = 7.52; %wing chord length from VSP in ft
bw_2 = 39.6; %Wing span from VSP in ft
bvt_2 = 6.2; %Vertical tail span in ft, from VSP

%sht_2 = 111.78291 %Area of horiz tail from VSP in ft^2
%svt_2 = 33.12426 %Area of Vert tail from VSP in ft^2
NoseToHT2 = 39.96; %Dist from Nose to Horiz. Tail in ft, VSP
NoseToVT2 = 41.6; %Dist from Nose to Vert. Tail in ft, VSP
l_ht2 = NoseToHT2 - cg2; %Estimation of length wise/horizontal moment arm
l_vt2 = NoseToVT2 - cg2; %Estimation of vertical moment arm
%v_ht2 = 0.9; %Horiz. tail volume coeff.  from table, c_h
%v_vt2 = 0.08; %Vert tail vol. coeff. from table, c_v
%s_ht2 = (v_ht2*sw_2*cw_2)/l_ht2 %horiz. tail area ft^2
%s_vt2 = (v_vt2*sw_2*bw_2)/l_vt2 %vert. tail area ft^2
%s_vt2 = 74.871; %VT area 37.355*2 from vsp
%s_ht2 = 65.59; %HT area from vsp
%v_ht2 = (s_ht2*l_ht2)/(sw_2*cw_2);
%v_vt2 = (s_vt2*l_vt2)/(bw_2*sw_2);
v_ht2 = 0.9; %Horiz. tail volume coeff.  from table 6.4, ref. [4]
v_vt2 = 0.08; %Vert tail vol. coeff. from table 6.4, ref. [4]
s_ht2 = (v_ht2*sw_2*cw_2)/l_ht2; %horiz. tail area ft^2
s_vt2 = (v_vt2*sw_2*bw_2)/l_vt2;
disp('TAIL PARAMETERS:')
fprintf('Horizontal tail area (S_ht) is:')
disp(s_ht2)
fprintf('Vertical tail area (S_vt) is:')
disp(s_vt2)
disp(' ')
disp('--------------------------------------------------')
%------------------------------------------------------------------------------------------%


%------------------------------------------------------------------------------------------%
%%
%Aspect ratio stuff
%ARw = ((bw_2)^2)/sw_2 %wing AR
ARw = 5.3; %Value given by chief egr.
ARvt = 4.0;  %tail AR, estimate given by chief engr.
disp('AR PARAMETERS:')
fprintf('Wing aspect ratio is:')
disp(ARw)
fprintf('Tail aspect ratio is:')
disp(ARvt)
disp(' ')
disp('--------------------------------------------------')
%------------------------------------------------------------------------------------------%


%------------------------------------------------------------------------------------------%
%%
%Static margin & neutral point calculations
aow = 0.11; %estimate for lift slope of wing ref. [10]
aos = 0.095; %estimate for stabilizer lift slope ref. [10]
aw = (ARw*aow)/(ARw + (18.25*aow)); %lift aoa of wing ref. [10]
ast = (ARvt*aos)/(ARvt + (18.25*aos)); %lift aoa of tail ref. [10]
xac = 11.12+0.25*cw_2; %AC of wing is 0.25 of mac, , estimate ref. [10]
ns = 0.6; %stabilzer efficiency, estimate from website listed above ref. [10]
de_da = 0.33; %estimate for downwash change w.r.t aoa, interpolation from ref.  [10]
xn = xac + (cw_2*(v_ht2*ns*(ast/aw))*(1-de_da)); %neutral point
SM = (xn - cg2)/cw_2; %static margin
disp('AIRCRAFT SM & Xn PARAMETERS:')
fprintf('Aircraft neutral point (xn) is:')
disp(xn)
fprintf('Aircraft Static Margin (SM) is:')
disp(SM)
fprintf('Aircraft SM in percent is:')
disp(SM*100)
disp(' ')
disp('--------------------------------------------------')
%------------------------------------------------------------------------------------------%


%------------------------------------------------------------------------------------------%
%%
%Aileron Sizing:
   clear;

M = 0.557;
Beta = sqrt(1-M.^2);
car_4 = 4.76;
car_e = atand((1/Beta)*tand(car_4));
lambda  = 5.87/9.03;
 

V = 628; % cruise velocity ft/s, by chief egr.
bw_2= 39.6; % span of wing in ft, given by chief egr/open vsp
sw_2 = 295; % Wing area ft^2
Clp = abs(-0.2529); % from pg.593 Nicolai ref. [1], damping roll coeff.
%determined lambda & beta, calculated car_4 and car_e, interpolated figure,
%solved for clp ref. [1]

delta_a = 20; %deflection in degrees down, assumption made based on pg 637 sadrey ref. [4]
Cl_alpha_w=  4.71385; %given by chief egr., lift coeff of wing
tau_a = 0.3273; %aileron effectiveness, pg 29 of sadrey fig. 12.12 ref. [4]

% equation of for chord at later section
% : c = (-23/15)*y + 69
% y1 = 25/12;
yi = 0.6*(bw_2/2); %start of outer wing, inboard position of aileron, estimate 60%, ref. [4]
yo = 0.85*(bw_2/2); %tip of wing, outboard position of aileron estimate 85%, ref. [4]
aileron_span = yo - yi;
syms y
c1 = 8.18; %inboard chord as a fxn of y, in ft
c2 = 11.043 - .2891919191*y; %outboard chord as a fxn of y, in ft, values by chief egr.

sectionAileron= int(c2*y,yi,yo); % + int(c1*y,y1,y2)
Cl_delta_a = (2*Cl_alpha_w*tau_a/(sw_2*bw_2))*double(sectionAileron);
% integral(c(y)ydy, y1,y2)

P = -2*(V/bw_2)*(Cl_delta_a/Clp)*deg2rad(delta_a);
Pdeg = -2*(V/bw_2)*(Cl_delta_a/Clp)*delta_a;
Pdeg_14 = Pdeg*1.4;

disp('AILERON PARAMETERS:')
disp(' ')
fprintf('Start aileron at: %0.1f feet from centerline\n',yi)
fprintf('End aileron at: %0.1f feet from centerline\n',yo)
fprintf('Cl_alpha_w (1/deg): %0.4f \n',Cl_alpha_w*pi/180)
fprintf('Cl_alpha_w (1/rad): %0.4f \n',Cl_alpha_w)
fprintf('tau: %0.1f feet\n',0.4)
fprintf('tau    |   chord %%\n')
fprintf('0.25   |   10\n')
fprintf('0.4    |   20\n')
fprintf('0.52   |   30\n')
fprintf('0.62   |   40\n')
fprintf('Max aileron deflection in degrees: %0.1f\n',delta_a)
fprintf('Roll Rate in radians/second: %0.1f\n',P)
fprintf('Roll Rate in degrees/second: %0.1f\n',Pdeg)
fprintf('Roll Rate in degrees/1.4second: %0.1f\n',Pdeg_14)
disp('For the given roll rate/1.4s, the aileron is 16.3% of c2') %reverse interpolated from sadrey to get tau
disp(' ')
fprintf('The aileron span is: %f', aileron_span)
disp(' ')
disp('--------------------------------------------------')
%------------------------------------------------------------------------------------------%


%------------------------------------------------------------------------------------------%
%%
%Flap Sizing:
%Using ref's [4] & [11]
%From sadreys pg. 667 ".. means that the flaps can start at the side of the fuselage..
%..and extend to the 60%  semispan station. However, with the application of spoilers, the.. 
%.. ailerons are generally reduced in size, and the flaps may extend to about 75% of the wing semispan"
%I will assume an initial estimate of 60% w/out knowledge of spoiler lengths
%According to ref [11], "Typically the flap depth cF amounts to 30% of the chord.."
cw_2 = 7.52; %chord length from VSP in ft
cs = 0.125*(bw_2/2); %flap span, ft
cf = 0.3*cw_2; %flap depth, in ft
disp('FLAP PARAMETERS')
disp(' ')
fprintf('FLAP Span in feet is:')
disp(cs)
fprintf('FLAP depth in feet is:')
disp(cf)
disp(' ')
disp('--------------------------------------------------')
%------------------------------------------------------------------------------------------%


%------------------------------------------------------------------------------------------%
%%
%Elevator Sizing Parameters:
%section 13.3.4
%------------------------------------------------------------------------------------------%
%%
%Rudder Sizing Parameters:
v_vt2 = 0.08; %Vert tail vol. coeff. from table 6.4, ref. [4]
%v_ht2 = (s_ht2*l_ht2)/(sw_2*cw_2);
%v_vt2 = (s_vt2*l_vt2)/(bw_2*sw_2);
bvt_2 = 6.2; %Vertical tail span in ft, from VSP
bht_2 = 12.5;
cg2 = 15.1086; %ft, aft from the nose
br_bv = (0.9);%avg of estimated values; rudder to verical stabilizer span
br = br_bv*bvt_2; %rudder span, based on Sadrey estimates of span to lift span ratio: br/bv = 0.7-1
NoseToVT2 = 41.6; %Dist from Nose to Vert. Tail in ft, VSP
l_vt2 = NoseToVT2 - cg2; %Estimation of vertical moment arm

density_1 = 0.001755*32.2; %for 10000 ft A.S.L, slug/ft^3
density_2 = 0.0019867*32.2; %for 6000 ft A.S.L
density_3 = 0.002175*32.2; %for 3000 ft A.S.L

V1 = 627.865; %design mission, cruise V is 372 knots = 627.865 ft/s, ipdr slides
V2 = 202.54; %Vto, takeoff speed, ft/s
V3 = 221*1.6878; %Vloiter, ft/s

Cl_alphav = 0.1; %lift slope coeff. at Vertical tail

q1 = 1/2 * density_1 * V1^2; %dynamic pressure
q2 = 1/2 * density_2 * V2^2; 
q3 = 1/2 * density_3 * V3^2; 

%n_v = qv/q; %vertical tail dynamic pressure ratio
n_v = 1.0; %est. based on ref [12]
dr = 30*pi/180; %rudder deflection in rad.

 
%^recommended to select a value equivalent to 80 percent of the stall as per Sadreys

%N_CG = -(1/2)*T*yt; %T is the thrust, which we assume an engine out thus 1/2 as per sadreys & yt = dist. b/t engine & CG
yt = 7.6; %in ft given by chf egr
yc = yt - bht_2/2;
yf = yt + bht_2/2;

%cndr is yawing moment coefficient with respect to rudder deflection
hp1engine = 5300*550/2; %Ibs
TL1 = hp1engine/V1;
TL2 = hp1engine/V2;
TL3 = hp1engine/V3;

cn_dr1 = -(TL1*yt)/(q1*sw_2*bw_2*dr*2); %cndr based on assymetric thrust, assuming da = 0, cn0 = 0, B = 0
tau_r1 = cn_dr1/(-Cl_alphav*v_vt2*n_v*br_bv); %AOA rudder effectiveness

cn_dr2 = -(TL2*yt)/(q2*sw_2*bw_2*dr*2); %cndr based on assymetric thrust, assuming da = 0, cn0 = 0, B = 0
tau_r2 = cn_dr2/(-Cl_alphav*v_vt2*n_v*br_bv); %AOA rudder effectiveness

cn_dr3 = -(TL3*yt)/(q3*sw_2*bw_2*dr*2); %cndr based on assymetric thrust, assuming da = 0, cn0 = 0, B = 0
tau_r3 = cn_dr3/(-Cl_alphav*v_vt2*n_v*br_bv); %AOA rudder effectiveness



v_vt2 = cn_dr2/(-Cl_alphav*n_v*tau_r2*br_bv)
s_vt2 = (v_vt2*sw_2*bw_2)/l_vt2
%cr_cv = ; %chord ratio of rudder chord to vertical tail chord, sadrey est. 0.15-0.4
%^^need tau_r to determine this quantity through tble 12.12 of sadreys
%if tau_r is >0.5 but <1.0 use cr_cv = 1 for all moving VT
%cv = ; chord length of VT
%cr = cv*cr_cv; %rudder chord length
cr = 4.23; %in ft, given by chf egr

%sv = ; %SA of VT
%sr_sv = ; %rudder SA to VT SA ratio, ranges b/t 0.15-0.35
%sr = sv*sr_sv; %rudder surface area
sr = 18.71775; % in ft & given by chf egr
disp('RUDDER PARAMETERS')
disp(' ')
fprintf('cndr1 is:')
disp(cn_dr1)
fprintf('taur1 is:')
disp(tau_r1)
disp(' ')
fprintf('cndr2 is:')
disp(cn_dr2)
fprintf('taur2 is:')
disp(tau_r2)
disp(' ')
fprintf('cndr3 is:')
disp(cn_dr3)
fprintf('taur3 is:')
disp(tau_r3)
disp('--------------------------------------------------')