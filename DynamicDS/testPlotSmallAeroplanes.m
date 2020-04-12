
figure;
r = [0,0,0];  % Heading, roll, pitch
plotSmallAeroplanes(gca, 0,0,0, r(1), r(2), r(3));
title('none');
axis equal;
view(3);

DCM = EulerToDCM_ENU(r(2), r(3), r(1));
[roll,pitch,heading] = DCMToEuler_ENU(DCM);


figure;
r = [0, deg2rad(20), 0];
plotSmallAeroplanes(gca, 0,0,0, r(1), r(2), r(3));
title('Roll 20');
axis equal;
view(3);

DCM = EulerToDCM_ENU(r(2), r(3), r(1));
[roll,pitch,heading] = DCMToEuler_ENU(DCM);

figure;
r = [0, 0, deg2rad(20)];
plotSmallAeroplanes(gca, 0,0,0, r(1), r(2), r(3));
title('Pitch 20');
axis equal;
view(3);

DCM = EulerToDCM_ENU(r(2), r(3), r(1));
[roll,pitch,heading] = DCMToEuler_ENU(DCM);

figure;
r = [deg2rad(45), 0, 0];
plotSmallAeroplanes(gca, 0,0,0, r(1), r(2), r(3));
title('Heading 45');
axis equal;
view(3);

DCM = EulerToDCM_ENU(r(2), r(3), r(1));
[roll,pitch,heading] = DCMToEuler_ENU(DCM);

figure;
r = [deg2rad(45), 0, deg2rad(20)];
plotSmallAeroplanes(gca, 0,0,0, r(1), r(2), r(3));
title('Heading 45, Pitch 20');
axis equal;
view(3);

DCM = EulerToDCM_ENU(r(2), r(3), r(1));
[roll,pitch,heading] = DCMToEuler_ENU(DCM);

figure;
r = [deg2rad(45), deg2rad(90), deg2rad(20)];
plotSmallAeroplanes(gca, 0,0,0, r(1), r(2), r(3));
title('Heading 45, Pitch 20, ROll 90');
axis equal;
view(3)

DCM = EulerToDCM_ENU(r(2), r(3), r(1));
[roll,pitch,heading] = DCMToEuler_ENU(DCM);
