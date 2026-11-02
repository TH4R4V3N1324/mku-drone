#Python Script to analyse data from a CSV file and generate summary statistics.

import pandas as pd
import numpy as np


#Configuration

csv_file = "flight7.csv" #CSV file name


#Load CSV

df = pd.read_csv(csv_file)

#Ommits any non nummerical data

numerical_df = df.select_dtypes(include=[np.number])

if numerical_df.empty:
    print("No numerical data found in the CSV file.")
    exit()

#Calculate statistics

results = []

for column in numerical_df.columns:

    data = numerical_df[column].dropna()  # Drops NaN values to provide accurate results

    mean = np.mean(data)
    rms = np.sqrt(np.mean(np.square(data)))
    std_dev = np.std(data, ddof=1)  # Sample standard deviation

    peak_positive = np.max(data)
    peak_negative = np.min(data)
    peam_to_peak = peak_positive - peak_negative

    results.append({
        "Column": column,
        "Mean": mean,
        "RMS": rms,
        "Standard Deviation": std_dev,
        "Peak Positive": peak_positive,
        "Peak Negative": peak_negative,
        "Peak to Peak": peam_to_peak
    })

#Display results

results_df = pd.DataFrame(results)

print("\nStatistic Analysis: ")
print(results_df.to_string(index=False))