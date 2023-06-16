

csvdat = readmatrix('WalkGait2FootSpace.csv');
x_raw = csvdat(:,2);
y_raw = csvdat(:,3);

figure
plot(x_raw, y_raw)

z_raw = zeros(numel(x_raw), 1);

writematrix([x_raw, y_raw, z_raw], 'walktest2-7pt.csv','Delimiter',',')