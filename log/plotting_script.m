clear all;
close all;

%% load new data 
static_mpu = load('static2g.txt');

figure('Name','MPU : Acceleration during static pose','NumberTitle','off');
subplot(3,1,1);
plot(static_mpu(:,1), static_mpu(:,2), 'g');
legend('mpu acceleration')
title('Acc_x during static period');

subplot(3,1,2);
plot(static_mpu(:,1), static_mpu(:,3), 'g');
legend('mpu acceleration')
title('Acc_y during static period');

subplot(3,1,3);
plot(static_mpu(:,1), static_mpu(:,4), 'g');
legend('mpu acceleration')
title('Acc_z during static period');

figure('Name','MPU additional hard filter : Gyroscope during vibrations','NumberTitle','off');
subplot(3,1,1);
plot(static_mpu(:,1), static_mpu(:,5), 'g');
legend('mpu acceleration');
title('Gyro_x during static period');

subplot(3,1,2);
plot(static_mpu(:,1), static_mpu(:,6), 'g');
legend('mpu acceleration');
title('Gyro_y during static period');

subplot(3,1,3);
plot(static_mpu(:,1), static_mpu(:,7), 'g');
legend('mpu acceleration');
title('Gyro_z during static period');

%% load new data : VIBRATE
vibrate_mpu = load('vibrate2g.txt');

figure('Name','MPU : Acceleration with perturbations','NumberTitle','off');
subplot(3,1,1);
plot(vibrate_mpu(:,1), vibrate_mpu(:,2), 'g');
legend('mpu acceleration')
title('Acc_x during vibrations');

subplot(3,1,2);
plot(vibrate_mpu(:,1), vibrate_mpu(:,3), 'g');
legend('mpu acceleration')
title('Acc_y during vibrations');

subplot(3,1,3);
plot(vibrate_mpu(:,1), vibrate_mpu(:,4), 'g');
legend('mpu acceleration')
title('Acc_z during vibrations');

figure('Name','MPU : Gyroscope with perturbations','NumberTitle','off');
subplot(3,1,1);
plot(vibrate_mpu(:,1), vibrate_mpu(:,5), 'g');
legend('mpu acceleration');
title('Gyro_x during vibrations');

subplot(3,1,2);
plot(vibrate_mpu(:,1), vibrate_mpu(:,6), 'g');
legend('mpu acceleration');
title('Gyro_y during vibrations');

subplot(3,1,3);
plot(vibrate_mpu(:,1), vibrate_mpu(:,7), 'g');
legend('mpu acceleration');
title('Gyro_z during vibrations');


%% load new data : random
random = load('random.txt');

figure('Name','MPU : Acceleration with random move','NumberTitle','off');
subplot(3,1,1);
plot(random(:,1), random(:,2), 'g');
legend('mpu acceleration')
title('Acc_x during vibrations');

subplot(3,1,2);
plot(random(:,1), random(:,3), 'g');
legend('mpu acceleration')
title('Acc_y during vibrations');

subplot(3,1,3);
plot(random(:,1), random(:,4), 'g');
legend('mpu acceleration')
title('Acc_z during vibrations');

figure('Name','MPU : Gyroscope with perturbations','NumberTitle','off');
subplot(3,1,1);
plot(random(:,1), random(:,5), 'g');
legend('mpu acceleration');
title('Gyro_x during vibrations');

subplot(3,1,2);
plot(random(:,1), random(:,6), 'g');
legend('mpu acceleration');
title('Gyro_y during vibrations');

subplot(3,1,3);
plot(random(:,1), random(:,7), 'g');
legend('mpu acceleration');
title('Gyro_z during vibrations');