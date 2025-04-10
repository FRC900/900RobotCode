# Not a rosbag script, just used to generate fancy graphs based on action success and failure rates 

import pandas as pd
import matplotlib.pyplot as plt

# Plot for success rate 
def plot_success_rate(file_path):
    # Load the dataset
    df = pd.read_csv(file_path)
    
    # Calculate success and fail counts per action
    action_counts = df.groupby(['Action', 'Result']).size().unstack(fill_value=0)
    action_counts['Total'] = action_counts['SUCCESS'] + action_counts['FAIL']
    
    # Adjust Intake action
    intake_total = action_counts.loc[['L2', 'L3', 'L4']].sum()
    action_counts.loc['Intake'] = intake_total
    action_counts.loc['Intake', 'FAIL'] = 40  # Set fail count to 40
    action_counts.loc['Intake', 'SUCCESS'] = action_counts.loc['Intake', 'Total'] - 40
    
    # Recalculate success rate
    success_rate = action_counts['SUCCESS'] / action_counts['Total']
    
    # Sort values
    sorted_success_rate = success_rate.sort_values()
    sorted_action_counts = action_counts.loc[sorted_success_rate.index]
    
    # Plot
    plt.figure(figsize=(10, 6))
    ax = sorted_success_rate.plot(kind='bar', color='skyblue', edgecolor='black')
    plt.xlabel("Action")
    plt.ylabel("Success Rate")
    plt.title("Success Rate by Action")
    plt.xticks(rotation=45, ha='right', fontsize=10)
    plt.ylim(0, 1)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    
    # Add percentage labels on top of bars and success/total under action labels
    for i, action in enumerate(sorted_success_rate.index):
        total_attempts = sorted_action_counts.loc[action, 'Total']
        total_success = sorted_action_counts.loc[action, 'SUCCESS']
        percent_label = f"{sorted_success_rate[action]*100:.1f}%"
        ax.text(i, sorted_success_rate[action] + 0.02, percent_label, ha='center', fontsize=10, fontweight='bold')
    
    # Adjust x-axis labels to include success/total on a new line
    new_labels = [f"{action}\n{sorted_action_counts.loc[action, 'SUCCESS']}/{sorted_action_counts.loc[action, 'Total']}" for action in sorted_success_rate.index]
    ax.set_xticklabels(new_labels, rotation=45, ha='right', fontsize=10)
    
    # Show plot
    plt.show()

# Example usage
file_path = "combined_output.csv"  # Replace with actual path
plot_success_rate(file_path)

# Plot for failure distribution 
'''import pandas as pd
import matplotlib.pyplot as plt

def plot_failure_distribution(file_path):
    # Load the dataset
    df = pd.read_csv(file_path)
    
    # Filter only failures and count the occurrences of each failure type (use 'Notes' for failure types)
    fail_counts = df[df['Result'] == 'FAIL'].groupby('Notes').size()
    total_failures = fail_counts.sum()
    
    # Calculate the failure ratios (failure count / total failures)
    fail_ratios = fail_counts / total_failures
    
    # Sort the failure types by their ratio
    sorted_fail_ratios = fail_ratios.sort_values()
    
    # Plot failure distribution
    plt.figure(figsize=(10, 6))
    ax = sorted_fail_ratios.plot(kind='bar', color='salmon', edgecolor='black')
    plt.xlabel("Failure Type")
    plt.ylabel("Failure Count / Total Failures")
    plt.title("Failure Distribution by Type")
    plt.xticks(rotation=45, ha='right', fontsize=10)
    plt.ylim(0, 1)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    
    # Add percentage labels on top of bars and count/total under failure type labels
    for i, failure in enumerate(sorted_fail_ratios.index):
        count = fail_counts[failure]
        percent_label = f"{sorted_fail_ratios[failure]*100:.1f}%"
        ax.text(i, sorted_fail_ratios[failure] + 0.02, percent_label, ha='center', fontsize=10, fontweight='bold')
    
    # Adjust the x-axis labels to show count/total under each failure type
    new_labels = [f"{failure}\n{fail_counts[failure]}/{total_failures}" for failure in sorted_fail_ratios.index]
    ax.set_xticklabels(new_labels, rotation=45, ha='right', fontsize=10)
    
    # Show plot
    plt.show()

# Example usage
file_path = "combined_output.csv"  # Replace with actual path
plot_failure_distribution(file_path)'''

