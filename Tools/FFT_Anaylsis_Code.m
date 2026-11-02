folderList = "D:\FFT Analysis";

timeCol = 2;
signalCol = 8;

for folder = folderList
    files = dir(fullfile(folder, "*.csv"));

    for k = 1:numel(files)
        filePath = fullfile(folder, files(k).name);
        data = readtable(filePath);

        time = data{:, timeCol};
        x = data{:, signalCol};

        time = time(:);
        x = x(:);

        Fs = 1 / mean(diff(time));
        x = x - mean(x, "omitnan");

        n = numel(x);
        Y = fft(x);

        f = Fs * (0:floor(n/2)) / n;
        P2 = abs(Y/n);
        P1 = P2(1:numel(f));

        if mod(n,2) == 0
            P1(2:end-1) = 2 * P1(2:end-1);
        else
            P1(2:end) = 2 * P1(2:end);
        end

        figure
        plot(f,P1)
        grid on
        title("FFT: "+string(files(k).name))
        subtitle("Accel Z")
        xlabel("Frequency (Hz)")
        ylabel("Amplitude")
    end
end
