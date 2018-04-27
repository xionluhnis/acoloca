% spectrogram stuff

% ambiant noise
ambiant = dlmread('data/ambiant.log');
ambiant = ambiant(:, 3);

% chirp from ADC (bad sampling)
chirps = dlmread('data/chirps.log');
chirps = chirps(:, 3);

% the exact pre-ADC from oscilloscope
chirps2 = dlmread('data/chirp.txt');

% doubling sampling
chirps3 = dlmread('data/chirps_new.log');

% tripled sampling
chirps4 = dlmread('data/chirps_new2.log');

% bitstream sampling
chirps5 = dlmread('data/chirps_new3.log');

data = { ambiant, chirps, chirps2, chirps3, chirps4, chirps5 };
spec = cell(numel(data), 2);
name = { 'ambiant', 'chirps', 'chirps2', 'chirps3', 'chirps4', 'chirps5' };

for i = 1:numel(data)
    s = spectrogram(data{i}, 128);
    s2 = spectrogram(normalize(data{i}, 3), 128);
    imagesc(abs(s));
    imagesc(abs(s2));
    imwrite(labelize(abs(s2)), parula, [name{i} '.png']);
    spec{i, 1} = abs(s);
    spec{i, 2} = abs(s2);
end